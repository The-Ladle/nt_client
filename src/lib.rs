#![warn(missing_docs)]
#![warn(rustdoc::missing_crate_level_docs)]

//! A blazingly fast [NetworkTables 4.1][NetworkTables] client.
//!
//! Provides a client that can be used to interface with a [NetworkTables] server. This is
//! intended to be used within a coprocessor on the robot. Keep in mind that this is a pre-1.0.0
//! release, so many things may not work properly and expect breaking changes until a full 1.0.0
//! release is available.
//!
//! # Examples
//!
//! ```no_run
//! use nt_client::{subscribe::ReceivedMessage, Client, NewClientOptions, NTAddr};
//!
//! # tokio_test::block_on(async {
//! let options = NewClientOptions { addr: NTAddr::Local, ..Default::default() };
//! let client = Client::new(options);
//!
//! client.connect_setup(setup).await.unwrap();
//! # });
//!
//! fn setup(client: &Client) {
//!     let thing_topic = client.topic("/thing");
//!     tokio::spawn(async move {
//!         let mut sub = thing_topic.subscribe(Default::default()).await.unwrap();
//!
//!         loop {
//!             match sub.recv().await {
//!                 Ok(ReceivedMessage::Updated((_topic, value))) => {
//!                     println!("topic updated: '{value}'");
//!                 },
//!                 Ok(_) => {},
//!                 Err(err) => {
//!                     eprintln!("{err}");
//!                     break;
//!                 },
//!             }
//!         }
//!     });
//! }
//! ```
//!
//! # Feature flags
//! - `struct`: adds support for struct parsing of common classes used in WPILib.
//! - `publish_bypass`: adds bypass versions of [`Topic::publish`] and [`Topic::generic_publish`] that do not wait for a server response.
//!   This is to serve as a workaround to [issue #7680][issue #7680].
//!   Once that bug is fixed, this feature will likely be deprecated and/or removed.
//! 
//! [NetworkTables]: https://github.com/wpilibsuite/allwpilib/blob/main/ntcore/doc/networktables4.adoc
//! [issue #7680]: https://github.com/wpilibsuite/allwpilib/issues/7680

use core::panic;
use std::{collections::VecDeque, convert::Into, error::Error, fmt::Debug, net::Ipv4Addr, sync::Arc, time::{Duration, Instant}};

use data::{BinaryData, ClientboundData, ClientboundTextData, PropertiesData, ServerboundMessage, ServerboundTextData, Subscribe, Unpublish, Unsubscribe};
use error::{ConnectError, ConnectionClosedError, IntoAddrError, PingError, ReceiveMessageError, ReconnectError, SendMessageError, UpdateTimeError};
use futures_util::{stream::{SplitSink, SplitStream}, Future, SinkExt, StreamExt, TryStreamExt};
use time::ext::InstantExt;
use tokio::{net::TcpStream, select, sync::{broadcast, mpsc, Notify, RwLock}, task::JoinHandle, time::{interval, timeout}};
use tokio_tungstenite::{tungstenite::{self, http::{Response, Uri}, ClientRequestBuilder, Message}, MaybeTlsStream, WebSocketStream};
use topic::{collection::TopicCollection, AnnouncedTopic, AnnouncedTopics, Topic};
use tracing::{debug, error, info, trace, warn};

pub mod error;
pub mod data;
pub mod topic;
pub mod subscribe;
pub mod publish;

#[cfg(feature = "struct")]
pub mod r#struct;

pub(crate) type NTServerSender = broadcast::Sender<Arc<ServerboundMessage>>;
pub(crate) type NTServerReceiver = broadcast::Receiver<Arc<ServerboundMessage>>;

pub(crate) type NTClientSender = broadcast::Sender<Arc<ClientboundData>>;
pub(crate) type NTClientReceiver = broadcast::Receiver<Arc<ClientboundData>>;

/// A cheaply-clonable handle to the client used to create topics.
pub struct ClientHandle {
    pub(crate) time: Arc<RwLock<NetworkTablesTime>>,
    pub(crate) announced_topics: Arc<RwLock<AnnouncedTopics>>,

    pub(crate) send_ws: (NTServerSender, NTServerReceiver),
    pub(crate) recv_ws: (NTClientSender, NTClientReceiver),
}

impl Debug for ClientHandle {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("ClientHandle")
            .field("time", &self.time)
            .field("announced_topics", &self.announced_topics)
            .finish()
    }
}

impl Clone for ClientHandle {
    fn clone(&self) -> Self {
        Self {
            time: self.time.clone(),
            announced_topics: self.announced_topics.clone(),
            send_ws: (self.send_ws.0.clone(), self.send_ws.0.subscribe()),
            recv_ws: (self.recv_ws.0.clone(), self.recv_ws.0.subscribe()),
        }
    }
}

impl Default for ClientHandle {
    fn default() -> Self {
        Self::new()
    }
}

impl ClientHandle {
    fn new() -> Self {
        Self {
            time: Default::default(),
            announced_topics: Default::default(),

            send_ws: broadcast::channel(1024),
            recv_ws: broadcast::channel(1024),
        }
    }

    /// Returns the current `NetworkTablesTime` for this client.
    ///
    /// This can safely be used asynchronously and across different threads.
    pub fn time(&self) -> Arc<RwLock<NetworkTablesTime>> {
        self.time.clone()
    }

    /// Returns an announced topic from its id.
    pub async fn announced_topic_from_id(&self, id: i32) -> Option<AnnouncedTopic> {
        self.announced_topics.read().await.get_from_id(id).cloned()
    }

    /// Returns an announced topic from its name.
    pub async fn announced_topic_from_name(&self, name: &str) -> Option<AnnouncedTopic> {
        self.announced_topics.read().await.get_from_name(name).cloned()
    }

    /// Returns a new topic with a given name.
    pub fn topic(&self, name: impl ToString) -> Topic {
        Topic::new(name.to_string(), self.clone())
    }

    /// Returns a new collection of topics with the given names.
    pub fn topics(&self, names: Vec<String>) -> TopicCollection {
        TopicCollection::new(names, self.clone())
    }
}

/// The client used to interact with a `NetworkTables` server.
///
/// When this goes out of scope, the websocket connection is closed and no attempts to reconnect
/// will be made.
pub struct Client {
    addr: Ipv4Addr,
    options: NewClientOptions,

    handle: ClientHandle,
}

impl Debug for Client {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("Client")
            .field("addr", &self.addr)
            .field("options", &self.options)
            .field("handle", &self.handle)
            .finish()
    }
}

impl AsRef<ClientHandle> for Client {
    fn as_ref(&self) -> &ClientHandle {
        &self.handle
    }
}

impl Client {
    /// Creates a new `Client` with options.
    ///
    /// # Panics
    /// Panics if the [`NTAddr::TeamNumber`] team number is greater than 25599.
    pub fn new(options: NewClientOptions) -> Self {
        let addr = match options.addr.clone().into_addr() {
            Ok(addr) => addr,
            Err(err) => panic!("{err}"),
        };

        Client {
            addr,
            options,

            handle: Default::default(),
        }
    }

    /// Returns the client handle.
    ///
    /// This can be cheaply cloned.
    pub fn handle(&self) -> &ClientHandle {
        &self.handle
    }

    /// Returns the current `NetworkTablesTime` for this client.
    ///
    /// This can safely be used asynchronously and across different threads.
    pub fn time(&self) -> Arc<RwLock<NetworkTablesTime>> {
        self.handle.time()
    }

    /// Returns an announced topic from its id.
    pub async fn announced_topic_from_id(&self, id: i32) -> Option<AnnouncedTopic> {
        self.handle.announced_topic_from_id(id).await
    }

    /// Returns an announced topic from its name.
    pub async fn announced_topic_from_name(&self, name: &str) -> Option<AnnouncedTopic> {
        self.handle.announced_topic_from_name(name).await
    }

    /// Returns a new topic with a given name.
    pub fn topic(&self, name: impl ToString) -> Topic {
        self.handle.topic(name)
    }

    /// Returns a new collection of topics with the given names.
    pub fn topics(&self, names: Vec<String>) -> TopicCollection {
        self.handle.topics(names)
    }

    /// Connects to the `NetworkTables` server.
    ///
    /// This future will only complete when the client has disconnected from the server.
    ///
    /// # Errors
    /// Returns an error if something goes wrong when connected to the server.
    pub async fn connect(self) -> Result<(), ConnectError> {
        self.connect_setup(|_| {}).await
    }

    /// Connects the the `NetworkTables` server, calling a setup function once the connection is made.
    ///
    /// This future will only complete when the client has disconnected from the server.
    ///
    /// # Errors
    /// Returns an error if something goes wrong when connected to the server.
    pub async fn connect_setup<F>(self, setup: F) -> Result<(), ConnectError>
    where F: FnOnce(&Self)
    {
        let (ws_stream, _) = if let Some(secure_port) = self.options.secure_port {
            match self.try_connect("wss", secure_port).await {
                Ok(ok) => ok,
                Err(tungstenite::Error::Io(_)) => self.try_connect("ws", self.options.unsecure_port).await?,
                Err(err) => return Err(err.into()),
            }
        } else {
            self.try_connect("ws", self.options.unsecure_port).await?
        };

        setup(&self);

        let handle = self.handle;

        let (write, read) = ws_stream.split();

        let pong_notify_recv = Arc::new(Notify::new());
        let pong_notify_send = pong_notify_recv.clone();
        let ping_task = Client::start_ping_task(pong_notify_recv, handle.send_ws.0.clone(), self.options.ping_interval, self.options.response_timeout);

        let (update_time_sender, update_time_recv) = mpsc::channel(1);
        let update_time_task = Client::start_update_time_task(self.options.update_time_interval, handle.time(), handle.send_ws.0.clone(), update_time_recv);

        let announced_topics = handle.announced_topics.clone();
        let write_task = Client::start_write_task(handle.send_ws.1, write);
        let read_task = Client::start_read_task(read, update_time_sender, pong_notify_send, announced_topics, handle.recv_ws.0);

        let result = select! {
            task = ping_task => task?.map_err(|err| err.into()),
            task = write_task => task?.map_err(|err| err.into()),
            task = read_task => task?.map_err(|err| err.into()),
            task = update_time_task => task?.map_err(|err| err.into()),
        };
        info!("closing connection");
        result
    }

    async fn try_connect(
        &self,
        scheme: &str,
        port: u16,
    ) -> Result<(WebSocketStream<MaybeTlsStream<TcpStream>>, Response<Option<Vec<u8>>>), tungstenite::Error> {
        let uri: Uri = format!("{scheme}://{}:{port}/nt/{}", self.addr, self.options.name).try_into().expect("valid websocket uri");
        let conn_str = uri.to_string();
        debug!("attempting connection at {conn_str}");
        let client_request = ClientRequestBuilder::new(uri)
            .with_sub_protocol("v4.1.networktables.first.wpi.edu");

        let res = tokio_tungstenite::connect_async(client_request).await;

        if res.is_ok() { info!("connected to server at {conn_str}") };

        res
    }

    fn start_ping_task(
        pong_recv: Arc<Notify>,
        ws_sender: NTServerSender,
        ping_interval: Duration,
        response_timeout: Duration,
    ) -> JoinHandle<Result<(), PingError>> {
        tokio::spawn(async move {
            let mut interval = interval(ping_interval);
            interval.tick().await;
            loop {
                interval.tick().await;
                ws_sender.send(ServerboundMessage::Ping.into()).map_err(|_| ConnectionClosedError)?;

                if (timeout(response_timeout, pong_recv.notified()).await).is_err() {
                    return Err(PingError::PongTimeout);
                }
            }
        })
    }

    fn start_update_time_task(
        update_time_interval: Duration,
        time: Arc<RwLock<NetworkTablesTime>>,
        ws_sender: NTServerSender,
        mut time_recv: mpsc::Receiver<(Duration, Duration)>,
    ) -> JoinHandle<Result<(), UpdateTimeError>> {
        tokio::spawn(async move {
            let mut interval = interval(update_time_interval);
            loop {
                interval.tick().await;

                let client_time = {
                    let time = time.read().await;
                    time.client_time()
                };
                let data = BinaryData::new::<u64>(
                    -1,
                    Duration::ZERO,
                    client_time.whole_microseconds().try_into().map_err(|_| UpdateTimeError::TimeOverflow)?,
                );
                ws_sender.send(ServerboundMessage::Binary(data).into()).map_err(|_| ConnectionClosedError)?;

                if let Some((timestamp, client_send_time)) = time_recv.recv().await {
                    let offset = {
                        let now = time.read().await.client_time();
                        let rtt = now - client_send_time;
                        let server_time = timestamp - rtt / 2;

                        server_time - now
                    };

                    let mut time = time.write().await;
                    time.offset = offset;
                    trace!("updated time, offset = {offset:?}");
                }
            }
        })
    }

    fn start_write_task(
        mut server_recv: NTServerReceiver,
        mut write: SplitSink<WebSocketStream<MaybeTlsStream<TcpStream>>, Message>,
    ) -> JoinHandle<Result<(), SendMessageError>> {
        tokio::spawn(async move {
            loop {
                match server_recv.recv().await {
                    Ok(message) => {
                        let packet = match &*message {
                            ServerboundMessage::Text(json) => {
                                match json {
                                    ServerboundTextData::Unpublish(Unpublish { pubuid }) => debug!("[pub {pubuid}] unpublished"),
                                    ServerboundTextData::Subscribe(Subscribe { topics, subuid, options }) => {
                                        debug!("[sub {subuid}] subscribed to {topics:?} with {options:?}");
                                    },
                                    ServerboundTextData::Unsubscribe(Unsubscribe { subuid }) => debug!("[sub {subuid}] unsubscribed"),
                                    _ => {},
                                };
                                serde_json::to_string(&[json]).map_err(|err| err.into()).map(Message::Text)
                            },
                            ServerboundMessage::Binary(binary) => {
                                if binary.id != -1 {
                                    debug!("[pub {}] set to {} at {:?}", binary.id, binary.data, binary.timestamp);
                                };
                                rmp_serde::to_vec(binary).map_err(|err| err.into()).map(Message::Binary)
                            },
                            ServerboundMessage::Ping => Ok(Message::Ping(Vec::new())),
                        };
                        match packet {
                            Ok(packet) => {
                                if !matches!(packet, Message::Ping(_)) { trace!("sent message: {packet:?}"); };
                                if write.send(packet).await.is_err() { return Err(SendMessageError::ConnectionClosed(ConnectionClosedError)); };
                            },
                            Err(err) => return Err(err),
                        };
                    },
                    Err(broadcast::error::RecvError::Closed) => return Err(SendMessageError::ConnectionClosed(ConnectionClosedError)),
                    Err(broadcast::error::RecvError::Lagged(amount)) => warn!("server writer lagged {amount}!"),
                };
            }
        })
    }

    fn start_read_task(
        read: SplitStream<WebSocketStream<MaybeTlsStream<TcpStream>>>,
        update_time_sender: mpsc::Sender<(Duration, Duration)>,
        pong_send: Arc<Notify>,
        announced_topics: Arc<RwLock<AnnouncedTopics>>,
        client_sender: NTClientSender,
    ) -> JoinHandle<Result<(), ReceiveMessageError>> {
        tokio::spawn(async move {
            read.err_into().try_for_each(|message| async {
                let message = match message {
                    Message::Binary(binary) => {
                        let mut binary = VecDeque::from(binary);
                        let mut binary_data = Vec::new();
                        while !binary.is_empty() {
                            let Ok(binary) = rmp_serde::from_read::<&mut VecDeque<u8>, BinaryData>(&mut binary) else {
                                warn!("malformed binary data");
                                continue;
                            };
                            if binary.id == -1 {
                                let Some(micros) = binary.data.as_u64() else {
                                    warn!("malformed timestamp data");
                                    continue;
                                };
                                let client_send_time = Duration::from_micros(micros);
                                if update_time_sender.send((binary.timestamp, client_send_time)).await.is_err() {
                                    return Err(ReceiveMessageError::ConnectionClosed(ConnectionClosedError));
                                };
                            }
                            binary_data.push(ClientboundData::Binary(binary));
                        };
                        Some(binary_data)
                    },
                    Message::Text(json) => {
                        match serde_json::from_str::<'_, Vec<ClientboundTextData>>(&json) {
                            Ok(text_data) => Some(text_data.into_iter().map(ClientboundData::Text).collect()),
                            Err(_) => {
                                warn!("malformed json data: {json}");
                                None
                            },
                        }
                    },
                    Message::Pong(_) => {
                        pong_send.notify_one();
                        None
                    },
                    Message::Close(_) => return Err(ReceiveMessageError::ConnectionClosed(ConnectionClosedError)),
                    _ => None,
                };

                if let Some(data_frame) = message {
                    trace!("received message(s): {data_frame:?}");
                    for data in data_frame {
                        match &data {
                            ClientboundData::Text(ClientboundTextData::Announce(announce)) => {
                                if let Some(pubuid) = announce.pubuid {
                                    debug!("[pub {pubuid}] publishing {:?}s to `{}` with {:?}", announce.r#type, announce.name, announce.properties);
                                } else {
                                    debug!("[topic {}] announced, type {:?} with {:?}", announce.name, announce.r#type, announce.properties);
                                }
                                let mut announced_topics = announced_topics.write().await;
                                announced_topics.insert(announce);
                            },
                            ClientboundData::Text(ClientboundTextData::Unannounce(unannounce)) => {
                                debug!("[topic {}] unannounced", unannounce.name);
                                let mut announced_topics = announced_topics.write().await;
                                announced_topics.remove(unannounce);
                            },
                            ClientboundData::Text(ClientboundTextData::Properties(PropertiesData { name, update, .. })) => {
                                let mut announced_topics = announced_topics.write().await;
                                let Some(topic) = announced_topics.get_mut_from_name(name) else {
                                    continue;
                                };

                                let properties = &mut topic.properties;
                                for (key, value) in update {
                                    match (key.as_ref(), value) {
                                        ("persistent", Some(serde_json::Value::Bool(persistent))) => properties.persistent = Some(*persistent),
                                        ("persistent", None) => properties.persistent = None,

                                        ("retained", Some(serde_json::Value::Bool(retained))) => properties.retained = Some(*retained),
                                        ("retained", None) => properties.retained = None,

                                        ("cached", Some(serde_json::Value::Bool(cached))) => properties.cached = Some(*cached),
                                        ("cached", None) => properties.cached = None,

                                        (key, Some(value)) => {
                                            properties.extra.insert(key.to_owned(), value.clone());
                                        },
                                        (key, None) => {
                                            properties.extra.remove(key);
                                        },
                                    };
                                };
                                debug!("[topic {name}] updated properties to {:?}", properties);
                            },
                            ClientboundData::Binary(BinaryData { id, timestamp, data, .. }) => {
                                let announced_topics = announced_topics.read().await;
                                if let Some(topic) = announced_topics.get_from_id(*id) {
                                    debug!("[topic {}] updated to {data} at {timestamp:?}", topic.name());
                                }
                            },
                        };

                        client_sender.send(data.into()).map_err(|_| ConnectionClosedError)?;
                    }
                };

                Ok(())
            }).await
        })
    }
}

/// Options when creating a new [`Client`].
#[derive(Debug, Clone, PartialEq, Eq, Hash)]
pub struct NewClientOptions {
    /// The address to connect to.
    /// 
    /// Default is [`NTAddr::Local`].
    pub addr: NTAddr,
    /// The port of the server.
    ///
    /// Default is `5810`.
    pub unsecure_port: u16,
    /// The port of the server. A value of [`None`] means that an attempt to make a secure
    /// connection will not be made.
    ///
    /// Default is `5811`.
    pub secure_port: Option<u16>,
    /// The name of the client.
    ///
    /// Default is `rust-client-{random u16}`
    pub name: String,
    /// The timeout for a server response.
    ///
    /// If this timeout gets exceeded when a server response is expected, such as in PING requests,
    /// the client will close the connection.
    ///
    /// Default is 1s.
    pub response_timeout: Duration,
    /// The interval at which to send ping messages.
    ///
    /// Default is 200ms.
    pub ping_interval: Duration,
    /// The interval at which to update server time.
    ///
    /// Default is 5s.
    pub update_time_interval: Duration,
}

impl Default for NewClientOptions {
    fn default() -> Self {
        Self {
            addr: Default::default(),
            unsecure_port: 5810,
            secure_port: Some(5811),
            name: format!("rust-client-{}", rand::random::<u16>()),
            response_timeout: Duration::from_secs(1),
            ping_interval: Duration::from_millis(200),
            update_time_interval: Duration::from_secs(5),
        }
    }
}

/// Represents an address that a `NetworkTables` client can connect to.
///
/// By default, this is set to [`NTAddr::Local`].
#[derive(Default, Debug, Clone, PartialEq, Eq, Hash)]
pub enum NTAddr {
    /// Address corresponding to an FRC team number.
    ///
    /// This should be used when deploying code to the robot.
    ///
    /// IP addresses are in the format of `10.TE.AM.2`.
    /// - Team 1: `10.0.1.2`
    /// - Team 12: `10.0.12.2`
    /// - Team 12345: `10.123.45.2`
    /// - Team 5071: `10.50.71.2`
    TeamNumber(u16),
    /// Local address (`127.0.0.1`).
    ///
    /// This should be used while in robot simulation mode.
    #[default]
    Local,
    /// Custom address.
    ///
    /// This is useful when the server is running simulate on a separate machine.
    Custom(Ipv4Addr),
}

impl NTAddr {
    /// Converts this into an [`Ipv4Addr`].
    ///
    /// # Errors
    /// Returns an error if the [`TeamNumber`][`NTAddr::TeamNumber`] team number is greater than 25599.
    pub fn into_addr(self) -> Result<Ipv4Addr, IntoAddrError> {
        let addr = match self {
            NTAddr::TeamNumber(team_number) => {
                if team_number > 25599 { return Err(IntoAddrError::InvalidTeamNumber(team_number)); };
                let first_section = team_number / 100;
                let last_two = team_number % 100;
                Ipv4Addr::new(10, first_section.try_into().unwrap(), last_two.try_into().unwrap(), 2)
            },
            NTAddr::Local => Ipv4Addr::LOCALHOST,
            NTAddr::Custom(addr) => addr,
        };
        Ok(addr)
    }
}

/// Time information about a `NetworkTables` server and client.
///
/// Provides methods to retrieve both the client's internal time and the server's time.
#[derive(Debug, Clone, PartialEq, Eq, Hash)]
pub struct NetworkTablesTime {
    started: Instant,
    offset: time::Duration,
}

impl Default for NetworkTablesTime {
    fn default() -> Self {
        Self::new()
    }
}

impl NetworkTablesTime {
    /// Creates a new `NetworkTablesTime` with the client start time of [`Instant::now`] and a
    /// server offset time of [`Duration::ZERO`].
    pub fn new() -> Self {
        Self { started: Instant::now(), offset: time::Duration::ZERO }
    }

    /// Returns the current client time.
    pub fn client_time(&self) -> time::Duration {
        Instant::now().signed_duration_since(self.started)
    }

    /// Returns the current server time.
    ///
    /// # Panics
    /// Panics if the calculated server time is negative. This should never happen and be reported
    /// if it does.
    pub fn server_time(&self) -> Duration {
        match (self.client_time() + self.offset).try_into() {
            Ok(duration) => duration,
            Err(_) => panic!("expected server time to be positive"),
        }
    }
}

/// Continuously calls `init` with a constructed [`Client`] whenever it returns an error,
/// effectively becoming a reconnect handler.
///
/// A return value of [`Ok`] means that `init` executed successfully and returned an
/// [`Ok`] value.
///
/// # Errors
/// If `init` returns a [`ReconnectError::Fatal`] variant, the inner error is returned.
///
/// # Examples
/// ```no_run
/// use nt_client::{subscribe::ReceivedMessage, error::ReconnectError, Client};
///
/// # tokio_test::block_on(async {
/// nt_client::reconnect(Default::default(), |client| async {
///     let topic = client.topic("/topic");
///     let sub_task = tokio::spawn(async move {
///         let mut subscriber = topic.subscribe(Default::default()).await.unwrap();
///
///         loop {
///             match subscriber.recv().await {
///                 Ok(ReceivedMessage::Updated((_, value))) => println!("updated: {value:?}"),
///                 Err(err) => return Err(err),
///                 _ => {},
///             }
///         };
///         Ok(())
///     });
///
///     // select! to make sure other tasks don't stay running
///     tokio::select! {
///         res = client.connect() => Ok(res?),
///         res = sub_task => res
///             .map_err(|err| ReconnectError::Fatal(err.into()))?
///             .map_err(|err| ReconnectError::Nonfatal(err.into())),
///     }
/// }).await.unwrap();
/// # })
/// ```
pub async fn reconnect<F, I>(options: NewClientOptions, mut init: I) -> Result<(), Box<dyn Error + Send + Sync>>
where
    F: Future<Output = Result<(), ReconnectError>>,
    I: FnMut(Client) -> F,
{
    loop {
        match init(Client::new(options.clone())).await {
            Ok(_) => return Ok(()),
            Err(ReconnectError::Fatal(err)) => {
                error!("fatal error occurred: {err}");
                return Err(err);
            },
            Err(ReconnectError::Nonfatal(err)) => {
                error!("client crashed! {err}");
                info!("attempting to reconnect");
            },
        }
    }
}

pub(crate) async fn recv_until<T, F>(recv_ws: &mut NTClientReceiver, mut filter: F) -> Result<T, broadcast::error::RecvError>
where F: FnMut(Arc<ClientboundData>) -> Option<T>
{
    loop {
        if let Some(data) = filter(recv_ws.recv().await?) {
            return Ok(data);
        }
    };
}

pub(crate) async fn recv_until_async<T, F, Fu>(recv_ws: &mut NTClientReceiver, mut filter: F) -> Result<T, broadcast::error::RecvError>
where
    Fu: Future<Output = Option<T>>,
    F: FnMut(Arc<ClientboundData>) -> Fu,
{
    loop {
        if let Some(data) = filter(recv_ws.recv().await?).await {
            return Ok(data);
        }
    };
}


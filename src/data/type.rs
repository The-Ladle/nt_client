//! `NetworkTables` data values and types.

use serde::{de::Visitor, Deserialize, Deserializer, Serialize, Serializer};

// holy macro
macro_rules! impl_data_type {
    // T (and vec<T>) to data type with from<data type> -> T impl
    ($t: ty $([vec => $a: expr])? => $d: expr ; $v: ident @ $f: expr) => {
        impl_data_type!(@ $t, $d, [$v]{ $f });
        $( impl_data_type!(vec $t => $a; $v @ { $f }); )?
    };

    // T (and vec<T>) to data type with from<data type> -> T impl (block)
    ($t: ty $([vec => $a: expr])? => $d: expr ; $v: ident @ $f: block) => {
        impl_data_type!(@ $t, $d, [$v]$f);
        $( impl_data_type!(vec $t => $a; $v @ { $f }); )?
    };

    // int (and vec<int>) to data type
    ($t: ty $([vec => $a: expr])? => $d: expr ; i) => {
        impl_data_type!(@ $t, $d, [value] value.as_i64());
        $( impl_data_type!(vec $t => $a; value @ { value.as_i64().and_then(|value| value.try_into().ok()) }); )?
    };
    // uint (and vec<uint>) to data type
    ($t: ty $([vec => $a: expr])? => $d: expr ; u) => {
        impl_data_type!(@ $t, $d, [value] value.as_u64());
        $( impl_data_type!(vec $t => $a; value @ { value.as_u64().and_then(|value| value.try_into().ok()) }); )?
    };

    // some_wrapper(vec<T>) to data type with mapper from value to T
    (vec($i: ty) $t: ty => $d: expr ; $v: ident @ $c: block) => {
        impl_data_type!(@ $t, $d, [value]{
            let vec = value.as_array()?;
            vec.iter()
                .map(|$v| $c)
                .collect::<Option<$t>>()
        }, [this]{
            let vec: Vec<$i> = this.into();
            rmpv::Value::from_iter(vec)
        });
    };
    // vec<T> to data type with mapper from value to T
    (vec $i: ty => $d: expr ; $v: ident @ $c: block) => {
        impl_data_type!(@ Vec<$i>, $d, [value]{
            let vec = value.as_array()?;
            vec.iter()
                .map(|$v| $c)
                .collect::<Option<Vec<$i>>>()
        }, [this]{ rmpv::Value::from_iter(this) });
    };
    // some_wrapper(vec<u8>) to data type
    (bytes $t: ty => $d: expr) => {
        impl_data_type!(vec(u8) $t => $d ; value @ {
            value.as_u64().and_then(|value| value.try_into().ok())
        });
    };

    // INTERNAL generic impl with some from logic without custom into logic
    (@ $t: ty, $d: expr, [ $v: ident ] $a: expr) => {
        impl_data_type!(@ $t, $d, [$v]{ $a.and_then(|value| value.try_into().ok()) }, [this]{ this.into() });
    };
    // INTERNAL generic impl with custom from and into logic
    (@ $t: ty, $d: expr, [ $v: ident ] $f: block, [ $s: ident ] $i: block) => {
        impl NetworkTableData for $t {
            fn data_type() -> DataType {
                use DataType::*;
                $d
            }
            #[allow(unused_variables)]
            fn from_value($v: &rmpv::Value) -> Option<Self> {
                $f
            }
            fn into_value(self) -> rmpv::Value {
                let $s = self;
                $i
            }
        }
    };
}

macro_rules! transparent {
    ($(#[$m: meta])* $t: ident : $g: ty) => {
        transparent!(@ $t, $($m)*, $g);
    };
    ($(#[$m: meta])* $t: ident : vec $g: ty) => {
        transparent!(@ $t, $($m)*, Vec<$g>);
        transparent!(@vec $t, $g);
    };

    (@ $t: ident, $($m: meta)*, $g: ty) => {
        $(#[$m])*
        pub struct $t(pub $g);

        impl From<$t> for $g {
            fn from(value: $t) -> Self {
                value.0
            }
        }
        impl From<$g> for $t {
            fn from(value: $g) -> Self {
                Self(value)
            }
        }

        #[allow(clippy::from_over_into)]
        impl Into<rmpv::Value> for $t {
            fn into(self) -> rmpv::Value {
                self.0.into()
            }
        }
    };
    (@vec $t: ident, $i: ty) => {
        impl FromIterator<$i> for $t {
            fn from_iter<I: IntoIterator<Item = $i>>(iter: I) -> Self {
                iter.into_iter().collect::<Vec<$i>>().into()
            }
        }
    };
}

/// A data type understood by a `NetworkTables` server.
#[derive(Serialize, Deserialize, Debug, Clone, PartialEq, Eq)]
#[serde(rename_all = "lowercase")]
pub enum DataType {
    /// [`bool`] data type.
    Boolean,
    /// [`f64`] data type.
    Double,
    /// Any integer data type.
    ///
    /// This includes Rust types like [`u8`], [`i32`], and [`u16`].
    Int,
    /// [`f32`] data type.
    Float,
    /// [`String`] data type.
    String,
    /// JSON data type.
    ///
    /// Internally, this is stored as a [`String`].
    Json,
    /// Raw binary data type.
    Raw,
    /// RPC data type.
    ///
    /// Internally, this is stored as a [`Vec<u8>`].
    Rpc,
    /// MessagePack data type.
    ///
    /// This is generally used for nested data.
    Msgpack,
    /// Google Protocol Buffers data type.
    ///
    /// Internally, this is stored as a [`Vec<u8>`].
    Protobuf,
    /// [`Vec<bool>`] data type.
    #[serde(rename = "boolean[]")]
    BooleanArray,
    /// [`Vec<f64>`] data type.
    #[serde(rename = "double[]")]
    DoubleArray,
    /// A [`Vec`] of integers data type.
    ///
    /// This includes Rust types like [`Vec<u16>`], [`Vec<i8>`], and [`Vec<u64>`].
    #[serde(rename = "int[]")]
    IntArray,
    /// [`Vec<f32>`] data type.
    #[serde(rename = "float[]")]
    FloatArray,
    /// [`Vec<String>`] data type.
    #[serde(rename = "string[]")]
    StringArray,

    /// Vec of custom binary `struct` data types.
    #[cfg(feature = "struct")]
    #[serde(untagged, serialize_with = "serialize_struct_array", deserialize_with = "deserialize_struct_array")]
    StructArray(String),

    /// Custom binary `struct` data type.
    #[cfg(feature = "struct")]
    #[serde(untagged, serialize_with = "serialize_struct", deserialize_with = "deserialize_struct")]
    Struct(String),

    /// An unknown data type.
    #[serde(untagged)]
    Unknown(String),
}

impl DataType {
    /// Creates a new `DataType` from an id.
    ///
    /// Returns [`Option::None`] if no `DataType` could be found with that id.
    ///
    /// Data type ids are as follows:
    /// - `0`: [`Self::Boolean`]
    /// - `1`: [`Self::Double`]
    /// - `2`: [`Self::Int`]
    /// - `3`: [`Self::Float`]
    /// - `4`: [`Self::String`]
    /// - `5`: [`Self::Raw`], [`Self::Rpc`], [`Self::Msgpack`], or [`Self::Protobuf`]
    /// - `16`: [`Self::BooleanArray`]
    /// - `17`: [`Self::DoubleArray`]
    /// - `18`: [`Self::IntArray`]
    /// - `19`: [`Self::FloatArray`]
    /// - `20`: [`Self::StringArray`]
    pub fn from_id(id: u32) -> Option<Self> {
        use DataType as D;

        match id {
            0 => Some(D::Boolean),
            1 => Some(D::Double),
            2 => Some(D::Int),
            3 => Some(D::Float),
            4 => Some(D::String),
            5 => Some(D::Raw),
            16 => Some(D::BooleanArray),
            17 => Some(D::DoubleArray),
            18 => Some(D::IntArray),
            19 => Some(D::FloatArray),
            20 => Some(D::StringArray),

            _ => None,
        }
    }

    /// Returns this `DataType` as an id.
    ///
    /// Data type ids are as follows:
    /// - [`Self::Boolean`] : `0`
    /// - [`Self::Double`] : `1`
    /// - [`Self::Int`] : `2`
    /// - [`Self::Float`] : `3`
    /// - [`Self::String`] or [`Self::Json`] : `4`
    /// - [`Self::Raw`], [`Self::Rpc`], [`Self::Msgpack`], or [`Self::Protobuf`] : `5`
    /// - [`Self::BooleanArray`] : `16`
    /// - [`Self::DoubleArray`] : `17`
    /// - [`Self::IntArray`] : `18`
    /// - [`Self::FloatArray`] : `19`
    /// - [`Self::StringArray`] : `20`
    ///
    /// # Panics
    ///
    /// Panics if self is [`DataType::Unknown`].
    pub fn as_id(&self) -> u32 {
        use DataType as D;

        match self {
            D::Boolean => 0,
            D::Double => 1,
            D::Int => 2,
            D::Float => 3,
            D::String | D::Json => 4,
            D::Raw | D::Rpc | D::Msgpack | D::Protobuf => 5,
            D::BooleanArray => 16,
            D::DoubleArray => 17,
            D::IntArray => 18,
            D::FloatArray => 19,
            D::StringArray => 20,
            D::Unknown(_) => panic!("unknown data type"),

            #[cfg(feature = "struct")]
            D::Struct(_) | D::StructArray(_) => 5,
        }
    }
}

/// A piece of data that can be sent and received by a `NetworkTables` server.
pub trait NetworkTableData {
    /// Returns the `DataType` that this piece of data is.
    fn data_type() -> DataType;

    /// Creates a new piece of data from a generic `MessagePack` value.
    fn from_value(value: &rmpv::Value) -> Option<Self> where Self: Sized;

    /// Converts this into a generic `MessagePack` value.
    fn into_value(self) -> rmpv::Value;
}

transparent!(
    /// A JSON string.
    #[derive(Clone, Debug)]
    JsonString: String
);
transparent!(
    /// Raw binary data.
    #[derive(Clone, Debug)]
    RawData: vec u8
);
transparent!(
    /// Raw RPC data.
    #[derive(Clone, Debug)]
    Rpc: vec u8
);
transparent!(
    /// Raw protobuf data.
    #[derive(Clone, Debug)]
    Protobuf: vec u8
);

impl_data_type!(bool [vec => BooleanArray] => Boolean; value @ value.as_bool());
impl_data_type!(f64 [vec => DoubleArray] => Double; value @ value.as_f64());
impl_data_type!(i8 [vec => IntArray] => Int; i);
impl_data_type!(i16 [vec => IntArray] => Int; i);
impl_data_type!(i32 [vec => IntArray] => Int; i);
impl_data_type!(i64 [vec => IntArray] => Int; i);
impl_data_type!(u8 [vec => IntArray] => Int; u);
impl_data_type!(u16 [vec => IntArray] => Int; u);
impl_data_type!(u32 [vec => IntArray] => Int; u);
impl_data_type!(u64 [vec => IntArray] => Int; u);
impl_data_type!(f32 [vec => FloatArray] => Float; value @ value.as_f64().map(|num| num as f32));
impl_data_type!(String [vec => StringArray] => String; value @ value.as_str().map(|str| str.to_owned()));
impl_data_type!(JsonString => Json; value @ value.as_str().map(|str| JsonString(str.to_owned())));
impl_data_type!(bytes RawData => Raw);
impl_data_type!(bytes Rpc => Rpc);
impl_data_type!(rmpv::Value => Msgpack; value @ Some(value.clone()));
impl_data_type!(bytes Protobuf => Protobuf);

pub(super) fn serialize_as_u32<S>(data_type: &DataType, serializer: S) -> Result<S::Ok, S::Error>
where S: Serializer
{
    serializer.serialize_u32(data_type.as_id())
}

pub(super) fn deserialize_u32<'de, D>(deserializer: D) -> Result<DataType, D::Error>
where D: Deserializer<'de>
{
    deserializer.deserialize_u32(DataTypeVisitor)
}

pub(super) struct DataTypeVisitor;

impl Visitor<'_> for DataTypeVisitor {
    type Value = DataType;

    fn expecting(&self, formatter: &mut std::fmt::Formatter) -> std::fmt::Result {
        write!(formatter, "a valid type id")
    }

    fn visit_i64<E>(self, v: i64) -> Result<Self::Value, E>
    where E: serde::de::Error
    {
        self.visit_u64(v.try_into().map_err(E::custom)?)
    }

    fn visit_u64<E>(self, v: u64) -> Result<Self::Value, E>
    where E: serde::de::Error
    {
        DataType::from_id(v.try_into().map_err(E::custom)?).ok_or(E::custom(format!("{v} is not a valid type id")))
    }
}

#[cfg(feature = "struct")]
fn serialize_struct<S>(type_name: &str, serializer: S) -> Result<S::Ok, S::Error>
where S: Serializer
{
    serializer.serialize_str(&format!("struct:{type_name}"))
}

#[cfg(feature = "struct")]
fn deserialize_struct<'de, D>(deserializer: D) -> Result<String, D::Error>
where D: Deserializer<'de>
{
    deserializer.deserialize_identifier(StructDataVisitor)
}

#[cfg(feature = "struct")]
struct StructDataVisitor;

#[cfg(feature = "struct")]
impl Visitor<'_> for StructDataVisitor {
    type Value = String;

    fn expecting(&self, formatter: &mut std::fmt::Formatter) -> std::fmt::Result {
        write!(formatter, "a valid struct type")
    }

    fn visit_str<E>(self, v: &str) -> Result<Self::Value, E>
    where E: serde::de::Error
    {
        let (left, right) = v.split_once(":").ok_or_else(|| serde::de::Error::custom("expected colon in struct type parsing"))?;
        if left != "struct" { return Err(serde::de::Error::custom("expected struct type to be prefixed with `struct:`")); };
        Ok(right.to_owned())
    }
}

#[cfg(feature = "struct")]
fn serialize_struct_array<S>(type_name: &str, serializer: S) -> Result<S::Ok, S::Error>
where S: Serializer
{
    serializer.serialize_str(&format!("struct:{type_name}[]"))
}

#[cfg(feature = "struct")]
fn deserialize_struct_array<'de, D>(deserializer: D) -> Result<String, D::Error>
where D: Deserializer<'de>
{
    deserializer.deserialize_identifier(StructDataArrayVisitor)
}

#[cfg(feature = "struct")]
struct StructDataArrayVisitor;

#[cfg(feature = "struct")]
impl Visitor<'_> for StructDataArrayVisitor {
    type Value = String;

    fn expecting(&self, formatter: &mut std::fmt::Formatter) -> std::fmt::Result {
        write!(formatter, "a valid struct type")
    }

    fn visit_str<E>(self, v: &str) -> Result<Self::Value, E>
    where E: serde::de::Error
    {
        let (left, right) = v.split_once(":").ok_or_else(|| serde::de::Error::custom("expected colon in struct type parsing"))?;
        if left != "struct" { return Err(serde::de::Error::custom("expected struct type to be prefixed with `struct:`")); };
        right.strip_suffix("[]").map(str::to_owned).ok_or_else(|| serde::de::Error::custom("expected array type to end with `[]`"))
    }
}


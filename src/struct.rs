//! NetworkTables `struct` pack/unpack support.

use std::mem::MaybeUninit;

use byte::{ByteBuffer, ByteReader};

use crate::data::r#type::{DataType, NetworkTableData};

pub mod byte;

macro_rules! struct_data {
    ($(#[$m: meta])* $vis: vis struct $ident: ident ( $n: literal ) { $($(#[$fm: meta])* $f: ident : $ty: tt ),+ $(,)? }) => {
        $(#[$m])*
        #[derive(Debug, Clone, Copy, PartialEq)]
        $vis struct $ident {
        $(
            $(#[$fm])*
            pub $f: struct_data!(@ty $ty),
        )+
        }

        impl StructData for $ident {
            fn type_name() -> String {
                $n.to_owned()
            }

            fn pack(self, buf: &mut ByteBuffer) {
                $(
                struct_data!(@pack(buf) $ty, self.$f);
                )+
            }

            fn unpack(read: &mut ByteReader) -> Option<Self> {
                $(
                let $f = struct_data!(@unpack(read) $ty);
                )+
                Some(Self {
                $(
                    $f,
                )+
                })
            }
        }
    };

    // TYPE MATCHING
    (@ty $ty: ty) => { $ty };


    // PACK MATCHING
    (@pack($b: ident) i8 , $v: expr) => {
        $b.write_i8($v);
    };
    (@pack($b: ident) i16 , $v: expr) => {
        $b.write_i16($v);
    };
    (@pack($b: ident) i32 , $v: expr) => {
        $b.write_i32($v);
    };
    (@pack($b: ident) i64 , $v: expr) => {
        $b.write_i64($v);
    };
    (@pack($b: ident) isize , $v: expr) => {
        $b.write_isize($v);
    };

    (@pack($b: ident) f32 , $v: expr) => {
        $b.write_f32($v);
    };
    (@pack($b: ident) f64 , $v: expr) => {
        $b.write_f64($v);
    };

    (@pack($b: ident) $ty: ty , $v: expr) => {
        $v.pack($b);
    };


    // UNPACK MATCHING
    (@unpack($b: ident) i8) => {
        $b.read_i8()?
    };
    (@unpack($b: ident) i16) => {
        $b.read_i16()?
    };
    (@unpack($b: ident) i32) => {
        $b.read_i32()?
    };
    (@unpack($b: ident) i64) => {
        $b.read_i64()?
    };
    (@unpack($b: ident) isize) => {
        $b.read_isize()?
    };

    (@unpack($b: ident) f32) => {
        $b.read_f32()?
    };
    (@unpack($b: ident) f64) => {
        $b.read_f64()?
    };

    (@unpack($b: ident) $ty: ty) => {
        <$ty>::unpack($b)?
    };
}

/// Data that can be packed and unpacked from raw bytes, known as a `Struct` in WPILib.
pub trait StructData {
    /// Returns the type name of this struct.
    ///
    /// This name will match the actual type name in WPILib.
    fn type_name() -> String;

    /// Puts object contents to `buf`.
    fn pack(self, buf: &mut ByteBuffer);

    /// Deserializes an object from `buf`.
    fn unpack(read: &mut ByteReader) -> Option<Self> where Self: Sized;

    /// Puts an iterator of objects to `buf`.
    fn pack_iter(iter: impl IntoIterator<Item = Self>, buf: &mut ByteBuffer)
    where Self: Sized
    {
        for item in iter {
            item.pack(buf);
        }
    }

    /// Deserializes exactly `size` objects from `buf`.
    fn unpack_vec(read: &mut ByteReader, size: usize) -> Option<Vec<Self>>
    where Self: Sized
    {
        let mut vec = Vec::with_capacity(size);
        for _ in 0..size {
            vec.push(Self::unpack(read)?);
        }
        Some(vec)
    }

    /// Deserializes a fixed-size array of length `S` from `buf`.
    fn unpack_array<const S: usize>(read: &mut ByteReader) -> Option<[Self; S]>
    where Self: Sized
    {
        let mut arr = [const { MaybeUninit::uninit() }; S];
        for i in 0..S {
            match Self::unpack(read) {
                Some(data) => {
                    arr[i].write(data);
                },
                None => {
                    // unpacking failed, we need to manually drop every item that has been unpacked
                    arr.iter_mut().take(i).for_each(|elem| {
                        // SAFETY: it is guaranteed that the array up until this point has been
                        // successfully unpacked and is initialized
                        unsafe { elem.assume_init_drop(); };
                    });
                    return None;
                }
            }
        }
        // SAFETY: it is guaranteed that by this point, every value in `arr` has been initialized
        Some(arr.map(|data| unsafe { data.assume_init() }))
    }
}

impl<T: StructData> NetworkTableData for T {
    fn data_type() -> DataType {
        DataType::Struct(Self::type_name())
    }

    fn from_value(value: &rmpv::Value) -> Option<Self> where Self: Sized {
        match value {
            rmpv::Value::Binary(bytes) => Self::unpack(&mut ByteReader::new(bytes)),
            _ => None,
        }
    }

    fn into_value(self) -> rmpv::Value {
        let mut buf = ByteBuffer::new();
        self.pack(&mut buf);
        rmpv::Value::Binary(buf.into())
    }
}

struct_data! {
    /// Feedforward constants that model a simple arm.
    pub struct ArmFeedforward("ArmFeedforward") {
        /// The static gain in volts.
        k_s: f64,
        /// The gravity gain in volts.
        k_g: f64,
        /// The velocity gain in V/rad/s.
        k_v: f64,
        /// The acceleration gain in V/rad/s².
        k_a: f64,
        /// The period in seconds.
        d_t: f64,
    }
}

struct_data! {
    /// Robot chassis speeds.
    pub struct ChassisSpeeds("ChassisSpeeds") {
        /// The x velocity in m/s.
        velocity_x: f64,
        /// The y velocity in m/s.
        velocity_y: f64,
        /// The angular velocity in rad/s.
        omega: f64,
    }
}

/// Represents a hermite spline of degree 3.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct CubicHermiteSpline {
    /// The control vector for the initial point in the x dimension.
    pub x_initial_control_vector: [f64; 2],
    /// The control vector for the final point in the x dimension.
    pub x_final_control_vector: [f64; 2],
    /// The control vector for the initial point in the y dimension.
    pub y_initial_control_vector: [f64; 2],
    /// The control vector for the final point in the y dimension.
    pub y_final_control_vector: [f64; 2],
}

impl StructData for CubicHermiteSpline {
    fn type_name() -> String {
        "CubicHermiteSpline".to_owned()
    }

    fn pack(self, buf: &mut ByteBuffer) {
        buf.write_f64(self.x_initial_control_vector[0]);
        buf.write_f64(self.x_initial_control_vector[1]);

        buf.write_f64(self.x_final_control_vector[0]);
        buf.write_f64(self.x_final_control_vector[1]);

        buf.write_f64(self.y_initial_control_vector[0]);
        buf.write_f64(self.y_initial_control_vector[1]);

        buf.write_f64(self.y_final_control_vector[0]);
        buf.write_f64(self.y_final_control_vector[1]);
    }

    fn unpack(read: &mut ByteReader) -> Option<Self> {
        let x_initial_control_vector = [read.read_f64()?, read.read_f64()?];
        let x_final_control_vector = [read.read_f64()?, read.read_f64()?];
        let y_initial_control_vector = [read.read_f64()?, read.read_f64()?];
        let y_final_control_vector = [read.read_f64()?, read.read_f64()?];

        Some(Self {
            x_initial_control_vector,
            x_final_control_vector,
            y_initial_control_vector,
            y_final_control_vector,
        })
    }
}

struct_data! {
    /// Constants for a DC motor.
    pub struct DCMotor("DCMotor") {
        /// Voltage at which the motor constants were measured in volts.
        nominal_voltage: f64,
        /// Torque when stalled in newton meters.
        stall_torque: f64,
        /// Current draw when stalled in amps.
        stall_current: f64,
        /// Current draw under no load in amps.
        free_current: f64,
        /// Angular velocity under no load in rad/s.
        free_speed: f64,
    }
}

struct_data! {
    /// Feedforward constants that model a differential drive drivetrain.
    pub struct DifferentialDriveFeedforward("DifferentialDriveFeedforward") {
        /// The linear velocity gain in V/(m/s).
        velocity_linear: f64,
        /// The linear acceleration gain in V/(m/s²).
        acceleration_linear: f64,
        /// The angular velocity gain in V/(rad/s).
        velocity_angular: f64,
        /// The angular acceleration gain in V/(rad/s²).
        acceleration_angular: f64,
    }
}

struct_data! {
    /// Kinematics for a differential drive.
    pub struct DifferentialDriveKinematics("DifferentialDriveKinematics") {
        /// The differential drive track width in meters.
        track_width: f64,
    }
}

struct_data! {
    /// Represents wheel positions for a differential drive drivetrain.
    pub struct DifferentialDriveWheelPositions("DifferentialDriveWheelPositions") {
        /// Distance measured by the left side.
        left: f64,
        /// Distance measured by the right side.
        right: f64,
    }
}

struct_data! {
    /// Represents the wheel speeds for a differential drive drivetrain.
    pub struct DifferentialDriveWheelSpeeds("DifferentialDriveWheelSpeeds") {
        /// Speed of the left side of the robot in m/s.
        left: f64,
        /// Speed of the right side of the robot in m/s.
        right: f64,
    }
}

struct_data! {
    /// Represents the motor voltages for a differential drive drivetrain.
    pub struct DifferentialDriveWheelVoltages("DifferentialDriveWheelVoltages") {
        /// Left wheel voltage.
        left: f64,
        /// Right wheel voltage.
        right: f64,
    }
}

struct_data! {
    /// Feedforward constants that model a simple elevator.
    pub struct ElevatorFeedforward("ElevatorFeedforward") {
        /// The static gain in volts.
        k_s: f64,
        /// The gravity gain in volts.
        k_g: f64,
        /// The velocity gain in V/(m/s).
        k_v: f64,
        /// The acceleration gain in V/(m/s²).
        k_a: f64,
        /// The period in seconds.
        d_t: f64,
    }
}

struct_data! {
    /// Represents a 2D ellipse space containing translational, rotational, and scaling components.
    pub struct Ellipse2d("Ellipse2d") {
        /// The center of the ellipse.
        center: Pose2d,
        /// The x semi-axis.
        x_semi_axis: f64,
        /// The y semi-axis.
        y_semi_axis: f64,
    }
}

/// Represents a plant defined using state-space notation.
///
/// A plant is a mathematical model of a system's dynamics.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct LinearSystem<const S: usize, const I: usize, const O: usize> {
    /// The system matrix A.
    pub a: Matrix<S, S>,
    /// The system matrix B.
    pub b: Matrix<S, I>,
    /// The system matrix C.
    pub c: Matrix<O, S>,
    /// The system matrix D.
    pub d: Matrix<O, I>,
}

impl<const S: usize, const I: usize, const O: usize> StructData for LinearSystem<S, I, O,> {
    fn type_name() -> String {
        format!("LinearSystem__{}_{}_{}", S, I, O)
    }

    fn pack(self, buf: &mut ByteBuffer) {
        self.a.pack(buf);
        self.b.pack(buf);
        self.c.pack(buf);
        self.d.pack(buf);
    }

    fn unpack(read: &mut ByteReader) -> Option<Self> where Self: Sized {
        let a = Matrix::unpack(read)?;
        let b = Matrix::unpack(read)?;
        let c = Matrix::unpack(read)?;
        let d = Matrix::unpack(read)?;

        Some(Self { a, b, c, d })
    }
}

/// Represents a RxC matrix.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Matrix<const R: usize, const C: usize> {
    /// The data of the matrix, indexed as `data[row][col]`.
    pub data: [[f64; C]; R],
}

impl<const R: usize, const C: usize> StructData for Matrix<R, C> {
    fn type_name() -> String {
        format!("Matrix__{}_{}", R, C)
    }

    fn pack(self, buf: &mut ByteBuffer) {
        self.data.into_iter().flatten().for_each(|value| {
            buf.write_f64(value);
        });
    }

    fn unpack(read: &mut ByteReader) -> Option<Self> {
        let mut data = [[0.0; C]; R];
        for value in data.iter_mut().flatten() {
            *value = read.read_f64()?;
        }
        Some(Self { data })
    }
}

struct_data! {
    /// Kinematics for a mecanum drive.
    pub struct MecanumDriveKinematics("MecanumDriveKinematics") {
        /// The front-left wheel translation.
        front_left: Translation2d,
        /// The front-right wheel translation.
        front_right: Translation2d,
        /// The rear-right wheel translation.
        rear_left: Translation2d,
        /// The rear-left wheel translation.
        rear_right: Translation2d,
    }
}

struct_data! {
    /// Represents the wheel positions for a mecanum drive drivetrain.
    pub struct MecanumDriveWheelPositions("MecanumDriveWheelPositions") {
        /// Distance measured by the front-left wheel in meters.
        front_left: f64,
        /// Distance measured by the front-right wheel in meters.
        front_right: f64,
        /// Distance measured by the rear-left wheel in meters.
        rear_left: f64,
        /// Distance measured by the rear-right wheel in meters.
        rear_right: f64,
    }
}

struct_data! {
    /// Represents the wheel speeds for a mecanum drive drivetrain.
    pub struct MecanumDriveWheelSpeeds("MecanumDriveWheelSpeeds") {
        /// Speed of the front-left wheel in m/s.
        front_left: f64,
        /// Speed of the front-right wheel in m/s.
        front_right: f64,
        /// Speed of the rear-left wheel in m/s.
        rear_left: f64,
        /// Speed of the rear-right wheel in m/s.
        rear_right: f64,
    }
}

struct_data! {
    /// Represents a 2D pose containing translational and rotational elements.
    pub struct Pose2d("Pose2d") {
        /// The translation component of the transformation.
        translation: Translation2d,
        /// The rotational component of the transformation.
        rotation: Rotation2d,
    }
}

struct_data! {
    /// Represents a 3D pose containing translational and rotational elements.
    pub struct Pose3d("Pose3d") {
        /// The translation component of the transformation.
        translation: Translation3d,
        /// The rotational component of the transformation.
        rotation: Rotation3d,
    }
}

struct_data! {
    /// Represents a quaternion.
    pub struct Quaternion("Quaternion") {
        /// The w component of the quaternion.
        w: f64,
        /// The x component of the quaternion.
        x: f64,
        /// The y component of the quaternion.
        y: f64,
        /// The z component of the quaternion.
        z: f64,
    }
}

/// Represents a hermite spline of degree 5.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct QuinticHermiteSpline {
    /// The control vector for the initial point in the x dimension.
    pub x_initial: [f64; 3],
    /// The control vector for the final point in the x dimension.
    pub x_final: [f64; 3],
    /// The control vector for the initial point in the y dimension.
    pub y_initial: [f64; 3],
    /// The control vector for the final point in the y dimension.
    pub y_final: [f64; 3],
}

impl StructData for QuinticHermiteSpline {
    fn type_name() -> String {
        "QuinticHermiteSpline".to_owned()
    }

    fn pack(self, buf: &mut ByteBuffer) {
        buf.write_f64(self.x_initial[0]);
        buf.write_f64(self.x_initial[1]);
        buf.write_f64(self.x_initial[2]);

        buf.write_f64(self.x_final[0]);
        buf.write_f64(self.x_final[1]);
        buf.write_f64(self.x_final[2]);

        buf.write_f64(self.y_initial[0]);
        buf.write_f64(self.y_initial[1]);
        buf.write_f64(self.y_initial[2]);

        buf.write_f64(self.y_final[0]);
        buf.write_f64(self.y_final[1]);
        buf.write_f64(self.y_final[2]);
    }

    fn unpack(read: &mut ByteReader) -> Option<Self> {
        let x_initial = [read.read_f64()?, read.read_f64()?, read.read_f64()?];
        let x_final = [read.read_f64()?, read.read_f64()?, read.read_f64()?];
        let y_initial = [read.read_f64()?, read.read_f64()?, read.read_f64()?];
        let y_final = [read.read_f64()?, read.read_f64()?, read.read_f64()?];

        Some(Self {
            x_initial,
            x_final,
            y_initial,
            y_final,
        })
    }
}

struct_data! {
    /// Represents a 2D rectangular space containing translational, rotational, and scaling components.
    pub struct Rectangle2d("Rectangle2d") {
        /// The center of the rectangle.
        center: Pose2d,
        /// The x size component of the rectangle.
        x_width: f64,
        /// The y size component of the rectangle.
        y_width: f64,
    }
}

struct_data! {
    /// Represents a rotation in a 2D coordinate frame representd by a point on the unit circle.
    pub struct Rotation2d("Rotation2d") {
        /// The rotation in radians.
        value: f64,
    }
}

struct_data! {
    /// Represents a rotation in a 2D coordinate frame representd by a point on the unit circle.
    pub struct Rotation3d("Rotation3d") {
        /// The quaternion representation of the rotation.
        quaternion: Quaternion,
    }
}

struct_data! {
    /// Feedforward constants that model a simple permanent-magnet DC motor.
    pub struct SimpleMotorFeedforward("SimpleMotorFeedforward") {
        /// The static gain in volts.
        k_s: f64,
        /// The velocity gain in V/(units/s).
        k_v: f64,
        /// The acceleration gain in V/(units/s²).
        k_a: f64,
        /// The period in seconds.
        d_t: f64,
    }
}

/// Kinematics for a swerve drive with N modules.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct SwerveDriveKinematics<const N: usize> {
    /// The swerve module locations.
    pub modules: [Translation2d; N],
}

impl<const N: usize> StructData for SwerveDriveKinematics<N> {
    fn type_name() -> String {
        format!("SwerveDriveKinematics__{}", N)
    }

    fn pack(self, buf: &mut ByteBuffer) {
        Translation2d::pack_iter(self.modules, buf);
    }

    fn unpack(read: &mut ByteReader) -> Option<Self> {
        let modules = Translation2d::unpack_array(read)?;
        Some(Self { modules })
    }
}

struct_data! {
    /// Represents the state of one swerve module.
    pub struct SwerveModulePosition("SwerveModulePosition") {
        /// Distance measured by the wheel of the module in meters.
        distance: f64,
        /// Angle of the module.
        angle: Rotation2d,
    }
}

struct_data! {
    /// Represents the state of one swerve module.
    pub struct SwerveModuleState("SwerveModuleState") {
        /// The speed of the wheel of the module in m/s.
        speed: f64,
        /// The angle of the module.
        angle: Rotation2d,
    }
}

struct_data! {
    /// Represents a transformation for a Pose2d in the pose's frame.
    pub struct Transform2d("Transform2d") {
        /// The translation component of the transformation.
        translation: Translation2d,
        /// The rotational component of the transformation.
        rotation: Rotation2d,
    }
}

struct_data! {
    /// Represents a transformation for a Pose3d in the pose's frame.
    pub struct Transform3d("Transform3d") {
        /// The translation component of the transformation.
        translation: Translation3d,
        /// The rotational component of the transformation.
        rotation: Rotation3d,
    }
}

struct_data! {
    /// Represents a translation in 2D space.
    pub struct Translation2d("Translation2d") {
        /// The x component of the translation.
        x: f64,
        /// The y component of the translation.
        y: f64,
    }
}

struct_data! {
    /// Represents a translation in 2D space.
    pub struct Translation3d("Translation3d") {
        /// The x component of the translation.
        x: f64,
        /// The y component of the translation.
        y: f64,
        /// The z component of the translation.
        z: f64,
    }
}

struct_data! {
    /// Represents a change in distance along a 2D arc.
    pub struct Twist2d("Twist2d") {
        /// The linear "dx" component.
        dx: f64,
        /// The linear "dy" component.
        dy: f64,
        /// The linear "dtheta" component in radians.
        dtheta: f64,
    }
}

struct_data! {
    /// Represents a change in distance along a 3D arc.
    pub struct Twist3d("Twist3d") {
        /// The linear "dx" component.
        dx: f64,
        /// The linear "dy" component.
        dy: f64,
        /// The linear "dz" component.
        dz: f64,
        /// The rotation vector x component in radians.
        rx: f64,
        /// The rotation vector y component in radians.
        ry: f64,
        /// The rotation vector z component in radians.
        rz: f64,
    }
}

/// Represents an N-dimensional Vector.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Vector<const N: usize> {
    /// The rows of the vector.
    pub rows: [f64; N],
}

impl<const N: usize> StructData for Vector<N> {
    fn type_name() -> String {
        format!("Vector__{}", N)
    }

    fn pack(self, buf: &mut ByteBuffer) {
        for value in self.rows {
            buf.write_f64(value);
        }
    }

    fn unpack(read: &mut ByteReader) -> Option<Self> {
        let mut rows = [0.0; N];
        for value in rows.iter_mut() {
            *value = read.read_f64()?;
        }
        Some(Self { rows })
    }
}

#[cfg(test)]
mod tests {
    use std::sync::Arc;

    use lazy_static::lazy_static;

    use super::*;

    #[test]
    fn test_unpack_arr() {
        struct_data! {
            struct MyStruct("") {
                f: f64,
                u: i32,
            }
        }

        let bytes = [
            0x71, 0x3D, 0x0A, 0xD7, 0xA3, 0x70, 0x20, 0x40, 0xEC, 0xFF, 0xFF, 0xFF, // 8.22, -20
            0x85, 0xEB, 0x51, 0xB8, 0x1E, 0x85, 0xF3, 0xBF, 0x37, 0x00, 0x00, 0x00, // -1.22, 55
        ];

        let buf: ByteBuffer = bytes.into();

        assert_eq!(MyStruct::unpack_array::<1>(&mut buf.read()), Some([MyStruct { f: 8.22, u: -20 }]));
        assert_eq!(MyStruct::unpack_array::<2>(&mut buf.read()), Some([MyStruct { f: 8.22, u: -20 }, MyStruct { f: -1.22, u: 55 }]));
        assert_eq!(MyStruct::unpack_array::<3>(&mut buf.read()), None);

        const DATA_VALUE: i32 = 25;
        lazy_static! {
            static ref DATA: Arc<i32> = Arc::new(DATA_VALUE);
        };

        struct DropTest {
            data: Arc<i32>,
        }

        impl StructData for DropTest {
            fn type_name() -> String {
                String::new()
            }

            fn pack(self, buf: &mut ByteBuffer) {
                buf.write_i32(*self.data);
            }

            fn unpack(read: &mut ByteReader) -> Option<Self> {
                // don't actually read any data since we're using "global" data
                read.read_i32()?;
                // if successful unpack, increment global Rc reference count
                Some(Self { data: Arc::clone(&DATA) })
            }
        }

        let bytes: [u8; 16] = [0; 16]; // 4 * size_of::<i32> bytes

        let buf: ByteBuffer = bytes.into();

        {
            let arr = DropTest::unpack_array::<4>(&mut buf.read());
            assert!(arr.is_some());
            let arr = arr.unwrap();
            assert!(arr.iter().all(|num| *num.data == DATA_VALUE));
            assert!(arr.iter().all(|num| Arc::ptr_eq(&num.data, &DATA)));
            assert_eq!(Arc::strong_count(&DATA), 5); // 1 + 4 drop tests unpacked
        }

        assert_eq!(Arc::strong_count(&DATA), 1); // arr is dropped

        let arr = DropTest::unpack_array::<5>(&mut buf.read());
        assert!(arr.is_none());
        assert_eq!(Arc::strong_count(&DATA), 1); // failed unpacking, all should be dropped
    }

    #[test]
    fn test_armfeedforward() {
        let bytes = [
            0x66, 0x66, 0x66, 0x66, 0x66, 0x66, 0x02, 0x40,
            0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x59, 0x40,
            0x29, 0x5C, 0x8F, 0xC2, 0xF5, 0xA8, 0x20, 0x40,
            0xCD, 0xCC, 0xCC, 0xCC, 0xCC, 0xCC, 0x14, 0x40,
            0x9A, 0x99, 0x99, 0x99, 0x99, 0x99, 0xF1, 0x3F,
        ];

        let r#struct = ArmFeedforward {
            k_s: 2.3,
            k_a: 5.2,
            k_v: 8.33,
            d_t: 1.1,
            k_g: 100.0,
        };
        assert_eq!(ArmFeedforward::type_name(), "ArmFeedforward");
        test_struct(r#struct, &bytes);
    }

    #[test]
    fn test_chassisspeeds() {
        let bytes = [
            0xCD, 0xCC, 0xCC, 0xCC, 0xCC, 0x4C, 0x34, 0x40,
            0x9A, 0x99, 0x99, 0x99, 0x99, 0x99, 0x20, 0x40,
            0xB8, 0x1E, 0x85, 0xEB, 0x51, 0x38, 0x26, 0x40,
        ];

        let r#struct = ChassisSpeeds {
            velocity_x: 20.3,
            velocity_y: 8.3,
            omega: 11.11,
        };
        assert_eq!(ChassisSpeeds::type_name(), "ChassisSpeeds");
        test_struct(r#struct, &bytes);
    }

    #[test]
    fn test_cubichermitespline() {
        let bytes = [
            0x8F, 0xC2, 0xF5, 0x28, 0x5C, 0x8F, 0x0A, 0x40,
            0xB8, 0x1E, 0x85, 0xEB, 0x51, 0x38, 0x22, 0x40,
            0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0x40, 0xC0,
            0x33, 0x33, 0x33, 0x33, 0x33, 0x43, 0x87, 0x40,
            0x9A, 0x99, 0x99, 0x99, 0x99, 0x99, 0x01, 0x40,
            0xAE, 0x47, 0xE1, 0x7A, 0x14, 0xAE, 0xFF, 0xBF,
            0xCD, 0xCC, 0xCC, 0xCC, 0xCC, 0x0C, 0x56, 0x40,
            0x5C, 0x8F, 0xC2, 0xF5, 0x28, 0x9C, 0x46, 0x40,
        ];

        let r#struct = CubicHermiteSpline {
            y_initial_control_vector: [2.2, -1.98],
            x_initial_control_vector: [3.32, 9.11],
            y_final_control_vector: [88.2, 45.22],
            x_final_control_vector: [-33.0, 744.4],
        };
        assert_eq!(CubicHermiteSpline::type_name(), "CubicHermiteSpline");
        test_struct(r#struct, &bytes);
    }

    #[test]
    fn test_dcmotor() {
        let bytes = [
            0x9A, 0x99, 0x99, 0x99, 0x99, 0x99, 0x28, 0x40,
            0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xF0, 0x3F,
            0x9A, 0x99, 0x99, 0x99, 0x99, 0x99, 0x01, 0x40,
            0xD7, 0xA3, 0x70, 0x3D, 0x0A, 0xD7, 0x1D, 0x40,
            0x00, 0x00, 0x00, 0x00, 0x00, 0xD0, 0x74, 0x40,
        ];

        let r#struct = DCMotor {
            free_speed: 333.0,
            free_current: 7.46,
            nominal_voltage: 12.3,
            stall_current: 2.2,
            stall_torque: 1.0,
        };
        assert_eq!(DCMotor::type_name(), "DCMotor");
        test_struct(r#struct, &bytes);
    }

    #[test]
    fn test_differentialdrivefeedforward() {
        let bytes = [
            0x66, 0x66, 0x66, 0x66, 0x66, 0x66, 0x24, 0x40,
            0x71, 0x3D, 0x0A, 0xD7, 0xA3, 0x70, 0x32, 0x40,
            0xCD, 0xCC, 0xCC, 0xCC, 0xCC, 0xCC, 0x1C, 0x40,
            0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x10, 0x40,
        ];

        let r#struct = DifferentialDriveFeedforward {
            velocity_linear: 10.2,
            velocity_angular: 7.2,
            acceleration_angular: 4.0,
            acceleration_linear: 18.44,
        };
        assert_eq!(DifferentialDriveFeedforward::type_name(), "DifferentialDriveFeedforward");
        test_struct(r#struct, &bytes);
    }

    #[test]
    fn test_differentialdrivekinematics() {
        let bytes = [
            0x9A, 0x99, 0x99, 0x99, 0x99, 0x99, 0x09, 0x40,
        ];

        let r#struct = DifferentialDriveKinematics {
            track_width: 3.2,
        };
        assert_eq!(DifferentialDriveKinematics::type_name(), "DifferentialDriveKinematics");
        test_struct(r#struct, &bytes);
    }

    #[test]
    fn test_differentialdrivewheelpositions() {
        let bytes = [
            0xCD, 0xCC, 0xCC, 0xCC, 0xCC, 0xCC, 0x14, 0x40,
            0x71, 0x3D, 0x0A, 0xD7, 0xA3, 0x70, 0x14, 0x40,
        ];

        let r#struct = DifferentialDriveWheelPositions {
            left: 5.2,
            right: 5.11,
        };
        assert_eq!(DifferentialDriveWheelPositions::type_name(), "DifferentialDriveWheelPositions");
        test_struct(r#struct, &bytes);
    }

    #[test]
    fn test_differentialdrivewheelspeeds() {
        let bytes = [
            0x9A, 0x99, 0x99, 0x99, 0x99, 0x59, 0x8F, 0x40,
            0x00, 0x00, 0x00, 0x00, 0x00, 0x50, 0x7A, 0xC0,
        ];

        let r#struct = DifferentialDriveWheelSpeeds {
            left: 1003.2,
            right: -421.0,
        };
        assert_eq!(DifferentialDriveWheelSpeeds::type_name(), "DifferentialDriveWheelSpeeds");
        test_struct(r#struct, &bytes);
    }

    #[test]
    fn test_differentialdrivewheelvoltages() {
        let bytes = [
            0x9A, 0x99, 0x99, 0x99, 0x99, 0x99, 0x27, 0x40,
            0xCD, 0xCC, 0xCC, 0xCC, 0xCC, 0xCC, 0x1C, 0xC0,
        ];

        let r#struct = DifferentialDriveWheelVoltages {
            left: 11.8,
            right: -7.2,
        };
        assert_eq!(DifferentialDriveWheelVoltages::type_name(), "DifferentialDriveWheelVoltages");
        test_struct(r#struct, &bytes);
    }

    #[test]
    fn test_elevatorfeedforward() {
        let bytes = [
            0xCD, 0xCC, 0xCC, 0xCC, 0xCC, 0xCC, 0x1C, 0x40,
            0xCD, 0xCC, 0xCC, 0xCC, 0xCC, 0xCC, 0xF4, 0x3F,
            0x52, 0xB8, 0x1E, 0x85, 0xEB, 0x51, 0x19, 0x40,
            0x66, 0x66, 0x66, 0x66, 0x66, 0x66, 0x1C, 0x40,
            0x9A, 0x99, 0x99, 0x99, 0x99, 0x99, 0xC9, 0x3F,
        ];

        let r#struct = ElevatorFeedforward {
            k_s: 7.2,
            k_a: 7.1,
            k_v: 6.33,
            d_t: 0.2,
            k_g: 1.3,
        };
        assert_eq!(ElevatorFeedforward::type_name(), "ElevatorFeedforward");
        test_struct(r#struct, &bytes);
    }

    #[test]
    fn test_ellipse2d() {
        let bytes = [
            0x9A, 0x99, 0x99, 0x99, 0x99, 0x99, 0x20, 0x40,
            0x9A, 0x99, 0x99, 0x99, 0x99, 0x99, 0x01, 0x40,
            0x66, 0x66, 0x66, 0x66, 0x66, 0x86, 0x54, 0x40,
            0x66, 0x66, 0x66, 0x66, 0x66, 0x66, 0x20, 0x40,
            0x33, 0x33, 0x33, 0x33, 0x33, 0x33, 0x0B, 0x40,
        ];

        let r#struct = Ellipse2d {
            center: Pose2d { translation: Translation2d { x: 8.3, y: 2.2 }, rotation: Rotation2d { value: 82.1 } },
            y_semi_axis: 3.4,
            x_semi_axis: 8.2,
        };
        assert_eq!(Ellipse2d::type_name(), "Ellipse2d");
        test_struct(r#struct, &bytes);
    }

    #[test]
    fn test_linearsystem() {
        let bytes = [
            0x66, 0x66, 0x66, 0x66, 0x66, 0x66, 0x02, 0x40,
            0x9A, 0x99, 0x99, 0x99, 0x99, 0x99, 0xF1, 0x3F,
            0x66, 0x66, 0x66, 0x66, 0x66, 0x66, 0x12, 0x40,
            0xCD, 0xCC, 0xCC, 0xCC, 0xCC, 0xCC, 0x00, 0x40,
            0xCD, 0xCC, 0xCC, 0xCC, 0xCC, 0xCC, 0x18, 0x40,
            0xCD, 0xCC, 0xCC, 0xCC, 0xCC, 0xCC, 0x1A, 0x40,
            0x33, 0x33, 0x33, 0x33, 0x33, 0x33, 0xF3, 0x3F,
            0x9A, 0x99, 0x99, 0x99, 0x99, 0x99, 0x09, 0x40,
            0x66, 0x66, 0x66, 0x66, 0x66, 0x66, 0x43, 0x40,
            0x9A, 0x99, 0x99, 0x99, 0x99, 0x19, 0x48, 0xC0,
            0x9A, 0x99, 0x99, 0x99, 0x99, 0x19, 0x43, 0x40,
            0x00, 0x00, 0x00, 0x00, 0x00, 0xC0, 0x5B, 0xC0,
            0x66, 0x66, 0x66, 0x66, 0x66, 0x66, 0x20, 0x40,
            0x66, 0x66, 0x66, 0x66, 0x66, 0xC6, 0x5B, 0x40,
            0x00, 0x00, 0x00, 0x00, 0x00, 0xE0, 0x72, 0x40,
            0x9A, 0x99, 0x99, 0x99, 0x99, 0x99, 0x28, 0x40,
            0x9A, 0x99, 0x99, 0x99, 0x99, 0x19, 0x4D, 0x40,
            0x9A, 0x99, 0x99, 0x99, 0x99, 0x99, 0xF1, 0x3F,
            0x33, 0x33, 0x33, 0x33, 0x33, 0x33, 0x15, 0x40,
            0x66, 0x66, 0x66, 0x66, 0x66, 0xC6, 0x50, 0x40,
            0x9A, 0x99, 0x99, 0x99, 0x99, 0x89, 0x7D, 0x40,
            0x00, 0x00, 0x00, 0x00, 0x00, 0x28, 0x8A, 0x40,
            0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x52, 0x40,
            0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x46, 0x40,
            0xCD, 0xCC, 0xCC, 0xCC, 0xCC, 0xCC, 0x00, 0x40,
            0xCD, 0xCC, 0xCC, 0xCC, 0xCC, 0xE4, 0x81, 0x40,
            0x66, 0x66, 0x66, 0x66, 0x66, 0x66, 0x0A, 0x40,
            0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x20, 0x40,
            0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x26, 0xC0,
            0x9A, 0x99, 0x99, 0x99, 0x99, 0x99, 0x09, 0x40,
            0x9A, 0x99, 0x99, 0x99, 0x99, 0x19, 0x3A, 0x40,
            0x66, 0x66, 0x66, 0x66, 0x66, 0x66, 0x16, 0x40,
            0xCD, 0xCC, 0xCC, 0xCC, 0xCC, 0xAC, 0x53, 0x40,
            0x66, 0x66, 0x66, 0x66, 0x66, 0x66, 0x1C, 0x40,
            0x00, 0x00, 0x00, 0x00, 0x00, 0x40, 0x46, 0x40,
            0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x22, 0x40,
            0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x26, 0x40,
            0x66, 0x66, 0x66, 0x66, 0x66, 0xB6, 0x71, 0xC0,
            0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x56, 0xC0,
            0x71, 0x3D, 0x0A, 0xD7, 0xA3, 0x70, 0x20, 0x40,
        ];

        let r#struct = LinearSystem::<3, 5, 2> {
            a: Matrix { data: [[2.3, 1.1, 4.6], [2.1, 6.2, 6.7], [1.2, 3.2, 38.8]] },
            b: Matrix { data: [[-48.2, 38.2, -111.0, 8.2, 111.1], [302.0, 12.3, 58.2, 1.1, 5.3], [67.1, 472.6, 837.0, 72.0, 44.0]] },
            c: Matrix { data: [[2.1, 572.6, 3.3], [8.0, -11.0, 3.2]] },
            d: Matrix { data: [[26.1, 5.6, 78.7, 7.1, 44.5], [9.0, 11.0, -283.4, -88.0, 8.22]] },
        };
        assert_eq!(LinearSystem::<3, 5, 2>::type_name(), "LinearSystem__3_5_2");
        test_struct(r#struct, &bytes);
    }

    #[test]
    fn test_matrix() {
        let bytes = [
            0x66, 0x66, 0x66, 0x66, 0x66, 0x66, 0x20, 0xC0,
            0xAE, 0x47, 0xE1, 0x7A, 0x14, 0xAE, 0x16, 0x40,
            0x9A, 0x99, 0x99, 0x99, 0x99, 0x19, 0x48, 0x40,
            0xCD, 0xCC, 0xCC, 0xCC, 0xCC, 0xCC, 0x10, 0x40,
            0x71, 0x3D, 0x0A, 0xD7, 0xA3, 0x70, 0x14, 0x40,
            0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x23, 0x40,
        ];

        let r#struct = Matrix::<2, 3> {
            data: [[-8.2, 5.67, 48.2], [4.2, 5.11, 9.5]],
        };
        assert_eq!(Matrix::<2, 3>::type_name(), "Matrix__2_3");
        test_struct(r#struct, &bytes);
    }

    #[test]
    fn test_mecanumdrivekinematics() {
        let bytes = [
            0xCD, 0xCC, 0xCC, 0xCC, 0xCC, 0xCC, 0x00, 0xC0,
            0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x10, 0x40,
            0xCD, 0xCC, 0xCC, 0xCC, 0xCC, 0xCC, 0x10, 0x40,
            0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x10, 0x40,
            0xCD, 0xCC, 0xCC, 0xCC, 0xCC, 0xCC, 0x00, 0xC0,
            0x66, 0x66, 0x66, 0x66, 0x66, 0x66, 0x20, 0xC0,
            0xCD, 0xCC, 0xCC, 0xCC, 0xCC, 0xCC, 0x10, 0x40,
            0x66, 0x66, 0x66, 0x66, 0x66, 0x66, 0x20, 0xC0,
        ];

        let r#struct = MecanumDriveKinematics {
            front_right: Translation2d { x: 4.2, y: 4.0 },
            rear_right: Translation2d { x: 4.2, y: -8.2 },
            rear_left: Translation2d { x: -2.1, y: -8.2 },
            front_left: Translation2d { x: -2.1, y: 4.0 },
        };
        assert_eq!(MecanumDriveKinematics::type_name(), "MecanumDriveKinematics");
        test_struct(r#struct, &bytes);
    }

    #[test]
    fn test_mecanumdrivewheelpositions() {
        let bytes = [
            0x66, 0x66, 0x66, 0x66, 0x66, 0x66, 0x20, 0x40,
            0x33, 0x33, 0x33, 0x33, 0x33, 0xF3, 0x68, 0x40,
            0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x22, 0x40,
            0x33, 0x33, 0x33, 0x33, 0x33, 0x33, 0x36, 0xC0,
        ];

        let r#struct = MecanumDriveWheelPositions {
            front_right: 199.6,
            rear_right: -22.2,
            rear_left: 9.0,
            front_left: 8.2,
        };
        assert_eq!(MecanumDriveWheelPositions::type_name(), "MecanumDriveWheelPositions");
        test_struct(r#struct, &bytes);
    }

    #[test]
    fn test_mecanumdrivewheelspeeds() {
        let bytes = [
            0xCD, 0xCC, 0xCC, 0xCC, 0xCC, 0xCC, 0x18, 0x40,
            0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x12, 0x40,
            0x00, 0x00, 0x00, 0x00, 0x00, 0x88, 0x8B, 0x40,
            0xE1, 0x7A, 0x14, 0xAE, 0x47, 0xA1, 0x55, 0xC0,
        ];

        let r#struct = MecanumDriveWheelSpeeds {
            front_right: 4.5,
            rear_right: -86.52,
            rear_left: 881.0,
            front_left: 6.2,
        };
        assert_eq!(MecanumDriveWheelSpeeds::type_name(), "MecanumDriveWheelSpeeds");
        test_struct(r#struct, &bytes);
    }

    #[test]
    fn test_pose2d() {
        let bytes = [
            0xCD, 0xCC, 0xCC, 0xCC, 0xCC, 0x3C, 0x91, 0xC0,
            0x66, 0x66, 0x66, 0x66, 0x66, 0xEC, 0x81, 0xC0,
            0xC3, 0xF5, 0x28, 0x5C, 0x8F, 0xC2, 0x1D, 0x40,
        ];

        let r#struct = Pose2d {
            translation: Translation2d { x: -1103.2, y: -573.55 },
            rotation: Rotation2d { value: 7.44 },
        };
        assert_eq!(Pose2d::type_name(), "Pose2d");
        test_struct(r#struct, &bytes);
    }

    #[test]
    fn test_pose3d() {
        let bytes = [
            0x7B, 0x14, 0xAE, 0x47, 0xE1, 0x7A, 0x0C, 0x40,
            0x33, 0x33, 0x33, 0x33, 0x33, 0x33, 0x1B, 0x40,
            0x9A, 0x99, 0x99, 0x99, 0x99, 0x99, 0x11, 0x40,
            0x54, 0xF7, 0x96, 0x9A, 0x56, 0xC2, 0xD0, 0x3F,
            0xC6, 0x49, 0xC9, 0x78, 0x73, 0x58, 0xB6, 0x3F,
            0xC6, 0x49, 0xC9, 0x78, 0x73, 0x58, 0xA6, 0xBF,
            0x71, 0xC5, 0x14, 0xC6, 0x9E, 0xB9, 0xEE, 0x3F,
        ];

        let r#struct = Pose3d {
            translation: Translation3d { x: 3.56, y: 6.8, z: 4.4 },
            rotation: Rotation3d { quaternion: Quaternion { w: 0.2618614682831908, x: 0.08728715609439694, y: -0.04364357804719847, z: 0.9601587170383664 } },
        };
        assert_eq!(Pose3d::type_name(), "Pose3d");
        test_struct(r#struct, &bytes);
    }

    #[test]
    fn test_quaternion() {
        let bytes = [
            0xCD, 0xCC, 0xCC, 0xCC, 0xCC, 0xCC, 0x23, 0x40,
            0xCD, 0xCC, 0xCC, 0xCC, 0xCC, 0xCC, 0x00, 0x40,
            0x66, 0x66, 0x66, 0x66, 0x66, 0x66, 0x16, 0x40,
            0x66, 0x66, 0x66, 0x66, 0x66, 0xA6, 0x4B, 0x40,
        ];

        let r#struct = Quaternion {
            w: 9.9,
            x: 2.1,
            y: 5.6,
            z: 55.3,
        };
        assert_eq!(Quaternion::type_name(), "Quaternion");
        test_struct(r#struct, &bytes);
    }

    #[test]
    fn test_quintichermitespline() {
        let bytes = [
            0x33, 0x33, 0x33, 0x33, 0x33, 0x33, 0x15, 0x40,
            0x71, 0x3D, 0x0A, 0xD7, 0xA3, 0x70, 0x10, 0x40,
            0x85, 0xEB, 0x51, 0xB8, 0x1E, 0x05, 0x22, 0x40,
            0x7B, 0x14, 0xAE, 0x47, 0xE1, 0x7A, 0x84, 0x3F,
            0x66, 0x66, 0x66, 0x66, 0x66, 0x66, 0x20, 0x40,
            0xCD, 0xCC, 0xCC, 0xCC, 0xCC, 0x0C, 0x46, 0x40,
            0xA4, 0x70, 0x3D, 0x0A, 0xD7, 0xA3, 0x1A, 0x40,
            0xCD, 0xCC, 0xCC, 0xCC, 0xCC, 0xCC, 0x10, 0x40,
            0x33, 0x33, 0x33, 0x33, 0x33, 0x33, 0x1F, 0x40,
            0x9A, 0x99, 0x99, 0x99, 0x99, 0x99, 0x26, 0x40,
            0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x21, 0x40,
            0x66, 0x66, 0x66, 0x66, 0x66, 0x66, 0x12, 0x40,
        ];

        let r#struct = QuinticHermiteSpline {
            y_initial: [6.66, 4.2, 7.8],
            x_final: [0.01, 8.2, 44.1],
            y_final: [11.3, 8.5, 4.6],
            x_initial: [5.3, 4.11, 9.01],
        };
        assert_eq!(QuinticHermiteSpline::type_name(), "QuinticHermiteSpline");
        test_struct(r#struct, &bytes);
    }

    #[test]
    fn test_rectangle2d() {
        let bytes = [
            0xCD, 0xCC, 0xCC, 0xCC, 0xCC, 0xCC, 0x21, 0x40,
            0xCD, 0xCC, 0xCC, 0xCC, 0xCC, 0xCC, 0x2C, 0x40,
            0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
            0x66, 0x66, 0x66, 0x66, 0x66, 0x66, 0x20, 0x40,
            0x33, 0x33, 0x33, 0x33, 0x33, 0x33, 0x23, 0x40,
        ];

        let r#struct = Rectangle2d {
            x_width: 8.2,
            center: Pose2d { translation: Translation2d { x: 8.9, y: 14.4 }, rotation: Rotation2d { value: 0.0 } },
            y_width: 9.6,
        };
        assert_eq!(Rectangle2d::type_name(), "Rectangle2d");
        test_struct(r#struct, &bytes);
    }

    #[test]
    fn test_rotation2d() {
        let bytes = [
            0xE1, 0x7A, 0x14, 0xAE, 0x47, 0xE1, 0x20, 0x40,
        ];

        let r#struct = Rotation2d {
            value: 8.44,
        };
        assert_eq!(Rotation2d::type_name(), "Rotation2d");
        test_struct(r#struct, &bytes);
    }

    #[test]
    fn test_rotation3d() {
        let bytes = [
            0xF7, 0xC6, 0x57, 0x88, 0x7B, 0xA7, 0xC8, 0x3F,
            0xFC, 0x93, 0xE4, 0x36, 0xE8, 0x3F, 0xDC, 0x3F,
            0x50, 0x8C, 0x8F, 0xF3, 0xC7, 0x57, 0xE6, 0x3F,
            0xCA, 0x58, 0xBC, 0xED, 0x24, 0xF3, 0xE0, 0x3F,
        ];

        let r#struct = Rotation3d {
            quaternion: Quaternion { w: 0.19261116177909063, x: 0.44140057907708274, y: 0.6982154614492035, z: 0.5296806948924992 },
        };
        assert_eq!(Rotation3d::type_name(), "Rotation3d");
        test_struct(r#struct, &bytes);
    }

    #[test]
    fn test_simplemotorfeedforward() {
        let bytes = [
            0x9A, 0x99, 0x99, 0x99, 0x99, 0x99, 0xF1, 0x3F,
            0x1F, 0x85, 0xEB, 0x51, 0xB8, 0x1E, 0xE5, 0x3F,
            0xCD, 0xCC, 0xCC, 0xCC, 0xCC, 0x4C, 0x32, 0x40,
            0x7B, 0x14, 0xAE, 0x47, 0xE1, 0x7A, 0xD4, 0x3F,
        ];

        let r#struct = SimpleMotorFeedforward {
            k_s: 1.1,
            d_t: 0.32,
            k_v: 0.66,
            k_a: 18.3,
        };
        assert_eq!(SimpleMotorFeedforward::type_name(), "SimpleMotorFeedforward");
        test_struct(r#struct, &bytes);
    }

    #[test]
    fn test_swervedrivekinematics() {
        let bytes = [
            0x33, 0x33, 0x33, 0x33, 0x33, 0x33, 0x11, 0x40,
            0x66, 0x66, 0x66, 0x66, 0x66, 0x66, 0x1A, 0x40,
            0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x14, 0xC0,
            0x66, 0x66, 0x66, 0x66, 0x66, 0x66, 0x1A, 0x40,
            0x33, 0x33, 0x33, 0x33, 0x33, 0x33, 0x11, 0x40,
            0xCD, 0xCC, 0xCC, 0xCC, 0xCC, 0xCC, 0x23, 0xC0,
            0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x14, 0xC0,
            0xCD, 0xCC, 0xCC, 0xCC, 0xCC, 0xCC, 0x23, 0xC0,
        ];

        let r#struct = SwerveDriveKinematics::<4> {
            modules: [Translation2d { x: 4.3, y: 6.6 }, Translation2d { x: -5.0, y: 6.6 }, Translation2d { x: 4.3, y: -9.9 }, Translation2d { x: -5.0, y: -9.9 }],
        };
        assert_eq!(SwerveDriveKinematics::<4>::type_name(), "SwerveDriveKinematics__4");
        test_struct(r#struct, &bytes);
    }

    #[test]
    fn test_swervemoduleposition() {
        let bytes = [
            0xCD, 0xCC, 0xCC, 0xCC, 0xCC, 0xCC, 0x00, 0x40,
            0x48, 0xE1, 0x7A, 0x14, 0xAE, 0x47, 0xE9, 0x3F,
        ];

        let r#struct = SwerveModulePosition {
            angle: Rotation2d { value: 0.79 },
            distance: 2.1,
        };
        assert_eq!(SwerveModulePosition::type_name(), "SwerveModulePosition");
        test_struct(r#struct, &bytes);
    }

    #[test]
    fn test_swervemodulestate() {
        let bytes = [
            0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x16, 0x40,
            0x6F, 0x12, 0x83, 0xC0, 0xCA, 0x21, 0x09, 0x40,
        ];

        let r#struct = SwerveModuleState {
            speed: 5.5,
            #[allow(clippy::approx_constant)]
            angle: Rotation2d { value: 3.1415 },
        };
        assert_eq!(SwerveModuleState::type_name(), "SwerveModuleState");
        test_struct(r#struct, &bytes);
    }

    #[test]
    fn test_transform2d() {
        let bytes = [
            0xC3, 0xF5, 0x28, 0x5C, 0x8F, 0xC2, 0x09, 0x40,
            0xCD, 0xCC, 0xCC, 0xCC, 0xCC, 0xCC, 0x19, 0x40,
            0x71, 0x3D, 0x0A, 0xD7, 0xA3, 0x70, 0x22, 0x40,
        ];

        let r#struct = Transform2d {
            translation: Translation2d { x: 3.22, y: 6.45 },
            rotation: Rotation2d { value: 9.22 },
        };
        assert_eq!(Transform2d::type_name(), "Transform2d");
        test_struct(r#struct, &bytes);
    }

    #[test]
    fn test_transform3d() {
        let bytes = [
            0x33, 0x33, 0x33, 0x33, 0x33, 0x33, 0x03, 0xC0,
            0x7B, 0x14, 0xAE, 0x47, 0xE1, 0x7A, 0x17, 0x40,
            0x33, 0x33, 0x33, 0x33, 0x33, 0x33, 0x22, 0x40,
            0xC3, 0x48, 0x2E, 0x27, 0xAC, 0xB7, 0xE5, 0x3F,
            0x65, 0xE9, 0x61, 0x02, 0x39, 0xD4, 0xD3, 0x3F,
            0xB7, 0x8B, 0x0A, 0x68, 0xDA, 0x44, 0xE5, 0xBF,
            0x15, 0x19, 0xC8, 0x94, 0xF2, 0xC5, 0xA4, 0x3F,
        ];

        let r#struct = Transform3d {
            translation: Translation3d { x: -2.4, y: 5.87, z: 9.1 },
            rotation: Rotation3d { quaternion: Quaternion { w: 0.6786709561586338, x: 0.3098280452028546, y: -0.6646549255423143, z: 0.04057272020513573 } },
        };
        assert_eq!(Transform3d::type_name(), "Transform3d");
        test_struct(r#struct, &bytes);
    }

    #[test]
    fn test_translation2d() {
        let bytes = [
            0x33, 0x33, 0x33, 0x33, 0x33, 0x33, 0x20, 0x40,
            0x0A, 0xD7, 0xA3, 0x70, 0x3D, 0x0A, 0xF7, 0x3F,
        ];

        let r#struct = Translation2d {
            x: 8.1,
            y: 1.44,
        };
        assert_eq!(Translation2d::type_name(), "Translation2d");
        test_struct(r#struct, &bytes);
    }

    #[test]
    fn test_translation3d() {
        let bytes = [
            0x33, 0x33, 0x33, 0x33, 0x33, 0x33, 0x22, 0x40,
            0x9A, 0x99, 0x99, 0x99, 0x99, 0x99, 0x1B, 0x40,
            0xF6, 0x28, 0x5C, 0x8F, 0xC2, 0xF5, 0x10, 0x40,
        ];

        let r#struct = Translation3d {
            x: 9.1,
            y: 6.9,
            z: 4.24,
        };
        assert_eq!(Translation3d::type_name(), "Translation3d");
        test_struct(r#struct, &bytes);
    }

    #[test]
    fn test_twist2d() {
        let bytes = [
            0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x10, 0x40,
            0x48, 0xE1, 0x7A, 0x14, 0xAE, 0x47, 0xF5, 0x3F,
            0xCD, 0xCC, 0xCC, 0xCC, 0xCC, 0xAC, 0x52, 0x40,
        ];

        let r#struct = Twist2d {
            dx: 4.0,
            dy: 1.33,
            dtheta: 74.7,
        };
        assert_eq!(Twist2d::type_name(), "Twist2d");
        test_struct(r#struct, &bytes);
    }

    #[test]
    fn test_twist3d() {
        let bytes = [
            0x9A, 0x99, 0x99, 0x99, 0x99, 0x99, 0xD9, 0x3F,
            0x66, 0x66, 0x66, 0x66, 0x66, 0x66, 0x12, 0x40,
            0x71, 0x3D, 0x0A, 0xD7, 0xA3, 0x70, 0x20, 0x40,
            0x9A, 0x99, 0x99, 0x99, 0x99, 0x99, 0x15, 0x40,
            0x9A, 0x99, 0x99, 0x99, 0x99, 0x99, 0xE9, 0x3F,
            0x66, 0x66, 0x66, 0x66, 0x66, 0x66, 0xF6, 0x3F,
        ];

        let r#struct = Twist3d {
            ry: 0.8,
            rz: 1.4,
            dx: 0.4,
            dy: 4.6,
            dz: 8.22,
            rx: 5.4,
        };
        assert_eq!(Twist3d::type_name(), "Twist3d");
        test_struct(r#struct, &bytes);
    }

    #[test]
    fn test_vector() {
        let bytes = [
            0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x12, 0x40,
            0x33, 0x33, 0x33, 0x33, 0x33, 0x33, 0xE3, 0xBF,
            0xF6, 0x28, 0x5C, 0x8F, 0xC2, 0xF5, 0x1F, 0x40,
        ];

        let r#struct = Vector::<3> {
            rows: [4.5, -0.6, 7.99],
        };
        assert_eq!(Vector::<3>::type_name(), "Vector__3");
        test_struct(r#struct, &bytes);
    }

    fn test_struct<S>(s: S, matches: &[u8])
    where S: StructData + std::clone::Clone + std::fmt::Debug + std::cmp::PartialEq
    {
        let mut buf = ByteBuffer::new();
        s.clone().pack(&mut buf);

        let bytes: Vec<u8> = buf.into();
        assert_eq!(&bytes, matches);

        assert_eq!(S::unpack(&mut ByteReader::new(matches)), Some(s));
    }
}


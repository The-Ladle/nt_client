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
    pub struct ArmFeedForward("ArmFeedforward") {
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

// TODO:
// LinearSystem
// Matrix

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
    pub struct MecanumDriveWheelSpeedsStruct("MecanumDriveWheelSpeeds") {
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
pub struct QuinticHermiteSplineStruct {
    /// The control vector for the initial point in the x dimension.
    pub x_initial: [f64; 3],
    /// The control vector for the final point in the x dimension.
    pub x_final: [f64; 3],
    /// The control vector for the initial point in the y dimension.
    pub y_initial: [f64; 3],
    /// The control vector for the final point in the y dimension.
    pub y_final: [f64; 3],
}

impl StructData for QuinticHermiteSplineStruct {
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

// TODO:
// SwerveDriveKinematics

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

// TODO:
// Vector

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
    fn test_arm_ff() {
        let bytes = [0, 0, 0, 0, 0, 0, 12, 64, 102, 102, 102, 102, 102, 102, 36, 64, 154, 153, 153, 153, 153, 153, 9, 64, 113, 61, 10, 215, 163, 112, 32, 64, 154, 153, 153, 153, 153, 153, 185, 63];

        test_struct(ArmFeedForward {
            k_s: 3.5,
            k_g: 10.2,
            k_v: 3.2,
            k_a: 8.22,
            d_t: 0.1,
        }, &bytes);
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


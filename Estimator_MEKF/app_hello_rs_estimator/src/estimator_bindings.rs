/* automatically generated by rust-bindgen 0.65.1 */
use core::ffi::{c_int, c_short, c_ushort, c_long, c_uint, c_ulong, c_void, c_char, c_schar, c_uchar};

pub const _STDINT_H: u32 = 1;
pub const _FEATURES_H: u32 = 1;
pub const _DEFAULT_SOURCE: u32 = 1;
pub const __GLIBC_USE_ISOC2X: u32 = 0;
pub const __USE_ISOC11: u32 = 1;
pub const __USE_ISOC99: u32 = 1;
pub const __USE_ISOC95: u32 = 1;
pub const __USE_POSIX_IMPLICITLY: u32 = 1;
pub const _POSIX_SOURCE: u32 = 1;
pub const _POSIX_C_SOURCE: u32 = 200809;
pub const __USE_POSIX: u32 = 1;
pub const __USE_POSIX2: u32 = 1;
pub const __USE_POSIX199309: u32 = 1;
pub const __USE_POSIX199506: u32 = 1;
pub const __USE_XOPEN2K: u32 = 1;
pub const __USE_XOPEN2K8: u32 = 1;
pub const _ATFILE_SOURCE: u32 = 1;
pub const __USE_MISC: u32 = 1;
pub const __USE_ATFILE: u32 = 1;
pub const __USE_FORTIFY_LEVEL: u32 = 0;
pub const __GLIBC_USE_DEPRECATED_GETS: u32 = 0;
pub const __GLIBC_USE_DEPRECATED_SCANF: u32 = 0;
pub const _STDC_PREDEF_H: u32 = 1;
pub const __STDC_IEC_559__: u32 = 1;
pub const __STDC_IEC_559_COMPLEX__: u32 = 1;
pub const __STDC_ISO_10646__: u32 = 201706;
pub const __GNU_LIBRARY__: u32 = 6;
pub const __GLIBC__: u32 = 2;
pub const __GLIBC_MINOR__: u32 = 31;
pub const _SYS_CDEFS_H: u32 = 1;
pub const __glibc_c99_flexarr_available: u32 = 1;
pub const __WORDSIZE: u32 = 64;
pub const __WORDSIZE_TIME64_COMPAT32: u32 = 1;
pub const __SYSCALL_WORDSIZE: u32 = 64;
pub const __LONG_DOUBLE_USES_FLOAT128: u32 = 0;
pub const __HAVE_GENERIC_SELECTION: u32 = 1;
pub const __GLIBC_USE_LIB_EXT2: u32 = 0;
pub const __GLIBC_USE_IEC_60559_BFP_EXT: u32 = 0;
pub const __GLIBC_USE_IEC_60559_BFP_EXT_C2X: u32 = 0;
pub const __GLIBC_USE_IEC_60559_FUNCS_EXT: u32 = 0;
pub const __GLIBC_USE_IEC_60559_FUNCS_EXT_C2X: u32 = 0;
pub const __GLIBC_USE_IEC_60559_TYPES_EXT: u32 = 0;
pub const _BITS_TYPES_H: u32 = 1;
pub const __TIMESIZE: u32 = 64;
pub const _BITS_TYPESIZES_H: u32 = 1;
pub const __OFF_T_MATCHES_OFF64_T: u32 = 1;
pub const __INO_T_MATCHES_INO64_T: u32 = 1;
pub const __RLIM_T_MATCHES_RLIM64_T: u32 = 1;
pub const __STATFS_MATCHES_STATFS64: u32 = 1;
pub const __FD_SETSIZE: u32 = 1024;
pub const _BITS_TIME64_H: u32 = 1;
pub const _BITS_WCHAR_H: u32 = 1;
pub const _BITS_STDINT_INTN_H: u32 = 1;
pub const _BITS_STDINT_UINTN_H: u32 = 1;
pub const INT8_MIN: i32 = -128;
pub const INT16_MIN: i32 = -32768;
pub const INT32_MIN: i32 = -2147483648;
pub const INT8_MAX: u32 = 127;
pub const INT16_MAX: u32 = 32767;
pub const INT32_MAX: u32 = 2147483647;
pub const UINT8_MAX: u32 = 255;
pub const UINT16_MAX: u32 = 65535;
pub const UINT32_MAX: u32 = 4294967295;
pub const INT_LEAST8_MIN: i32 = -128;
pub const INT_LEAST16_MIN: i32 = -32768;
pub const INT_LEAST32_MIN: i32 = -2147483648;
pub const INT_LEAST8_MAX: u32 = 127;
pub const INT_LEAST16_MAX: u32 = 32767;
pub const INT_LEAST32_MAX: u32 = 2147483647;
pub const UINT_LEAST8_MAX: u32 = 255;
pub const UINT_LEAST16_MAX: u32 = 65535;
pub const UINT_LEAST32_MAX: u32 = 4294967295;
pub const INT_FAST8_MIN: i32 = -128;
pub const INT_FAST16_MIN: i64 = -9223372036854775808;
pub const INT_FAST32_MIN: i64 = -9223372036854775808;
pub const INT_FAST8_MAX: u32 = 127;
pub const INT_FAST16_MAX: u64 = 9223372036854775807;
pub const INT_FAST32_MAX: u64 = 9223372036854775807;
pub const UINT_FAST8_MAX: u32 = 255;
pub const UINT_FAST16_MAX: i32 = -1;
pub const UINT_FAST32_MAX: i32 = -1;
pub const INTPTR_MIN: i64 = -9223372036854775808;
pub const INTPTR_MAX: u64 = 9223372036854775807;
pub const UINTPTR_MAX: i32 = -1;
pub const PTRDIFF_MIN: i64 = -9223372036854775808;
pub const PTRDIFF_MAX: u64 = 9223372036854775807;
pub const SIG_ATOMIC_MIN: i32 = -2147483648;
pub const SIG_ATOMIC_MAX: u32 = 2147483647;
pub const SIZE_MAX: i32 = -1;
pub const WINT_MIN: u32 = 0;
pub const WINT_MAX: u32 = 4294967295;
pub const true_: u32 = 1;
pub const false_: u32 = 0;
pub const __bool_true_false_are_defined: u32 = 1;
pub const vec3d_size: u32 = 3;
pub const STABILIZER_NR_OF_MOTORS: u32 = 4;
pub const RATE_1000_HZ: u32 = 1000;
pub const RATE_500_HZ: u32 = 500;
pub const RATE_250_HZ: u32 = 250;
pub const RATE_100_HZ: u32 = 100;
pub const RATE_50_HZ: u32 = 50;
pub const RATE_25_HZ: u32 = 25;
pub const RATE_MAIN_LOOP: u32 = 1000;
pub const ATTITUDE_RATE: u32 = 500;
pub const POSITION_RATE: u32 = 100;
pub const RATE_HL_COMMANDER: u32 = 100;
pub const RATE_SUPERVISOR: u32 = 25;
pub const CONFIG_DECK_LIGHTHOUSE_MAX_N_BS: u32 = 4;
pub const CONFIG_DECK_FORCE: &[u8; 5usize] = b"none\0";
pub const CONFIG_DECK_LEDRING_DEFAULT_EFFECT: u32 = 6;
pub const CONFIG_PLATFORM_CF2: u32 = 1;
pub const CONFIG_SENSORS_BMI088_BMP3XX: u32 = 1;
pub const CONFIG_CPX_UART2_BAUDRATE: u32 = 576000;
pub const CONFIG_CONTROLLER_AUTO_SELECT: u32 = 1;
pub const CONFIG_MOTORS_ESC_PROTOCOL_ONESHOT125: u32 = 1;
pub const CONFIG_DECK_FLOW: u32 = 1;
pub const CONFIG_DECK_ACTIVE_MARKER: u32 = 1;
pub const CONFIG_ESTIMATOR_KALMAN_ENABLE: u32 = 1;
pub const CONFIG_DECK_MULTIRANGER: u32 = 1;
pub const CONFIG_DECK_LEDRING: u32 = 1;
pub const CONFIG_DECK_LIGHTHOUSE: u32 = 1;
pub const CONFIG_DECK_ZRANGER: u32 = 1;
pub const CONFIG_ENABLE_CPX: u32 = 1;
pub const CONFIG_DECK_OA: u32 = 1;
pub const CONFIG_DECK_USD: u32 = 1;
pub const CONFIG_DECK_RPM: u32 = 1;
pub const CONFIG_DEFRAG_STORAGE_ON_STARTUP: u32 = 1;
pub const CONFIG_DECK_LEDRING_NBR_LEDS: u32 = 12;
pub const CONFIG_DECK_LEDRING_DIMMER: u32 = 0;
pub const CONFIG_MOTORS_DEFAULT_BAT_TEST_PWM_RATIO: u32 = 0;
pub const CONFIG_SENSORS_MPU9250_LPS25H: u32 = 1;
pub const CONFIG_SENSORS_BMI088_I2C: u32 = 1;
pub const CONFIG_ENABLE_CPX_ON_UART2: u32 = 1;
pub const CONFIG_ESTIMATOR_OUTLIER_FILTERS: u32 = 1;
pub const CONFIG_ENABLE_THRUST_BAT_COMPENSATED: u32 = 1;
pub const CONFIG_DECK_LOCO: u32 = 1;
pub const CONFIG_DECK_LOCO_NR_OF_ANCHORS: u32 = 8;
pub const CONFIG_DECK_AI: u32 = 1;
pub const CONFIG_DECK_AI_WIFI_NO_SETUP: u32 = 1;
pub const CONFIG_POWER_DISTRIBUTION_QUADROTOR: u32 = 1;
pub const CONFIG_MOTORS_DEFAULT_PROP_TEST_PWM_RATIO: u32 = 0;
pub const CONFIG_ESTIMATOR_AUTO_SELECT: u32 = 1;
pub const CONFIG_DECK_ZRANGER2: u32 = 1;
pub const CONFIG_IMU_MAHONY_QUATERNION: u32 = 1;
pub const CONFIG_DECK_BUZZ: u32 = 1;
pub const CONFIG_CROSS_COMPILE: &[u8; 15usize] = b"arm-none-eabi-\0";
pub const CONFIG_DECK_LOCO_ALGORITHM_AUTO: u32 = 1;
pub type __u_char = c_uchar;
pub type __u_short = c_ushort;
pub type __u_int = c_uint;
pub type __u_long = c_ulong;
pub type __int8_t = c_schar;
pub type __uint8_t = c_uchar;
pub type __int16_t = c_short;
pub type __uint16_t = c_ushort;
pub type __int32_t = c_int;
pub type __uint32_t = c_uint;
pub type __int64_t = c_long;
pub type __uint64_t = c_ulong;
pub type __int_least8_t = __int8_t;
pub type __uint_least8_t = __uint8_t;
pub type __int_least16_t = __int16_t;
pub type __uint_least16_t = __uint16_t;
pub type __int_least32_t = __int32_t;
pub type __uint_least32_t = __uint32_t;
pub type __int_least64_t = __int64_t;
pub type __uint_least64_t = __uint64_t;
pub type __quad_t = c_long;
pub type __u_quad_t = c_ulong;
pub type __intmax_t = c_long;
pub type __uintmax_t = c_ulong;
pub type __dev_t = c_ulong;
pub type __uid_t = c_uint;
pub type __gid_t = c_uint;
pub type __ino_t = c_ulong;
pub type __ino64_t = c_ulong;
pub type __mode_t = c_uint;
pub type __nlink_t = c_ulong;
pub type __off_t = c_long;
pub type __off64_t = c_long;
pub type __pid_t = c_int;
#[repr(C)]
#[derive(Debug, Copy, Clone)]
pub struct __fsid_t {
    pub __val: [c_int; 2usize],
}
pub type __clock_t = c_long;
pub type __rlim_t = c_ulong;
pub type __rlim64_t = c_ulong;
pub type __id_t = c_uint;
pub type __time_t = c_long;
pub type __useconds_t = c_uint;
pub type __suseconds_t = c_long;
pub type __daddr_t = c_int;
pub type __key_t = c_int;
pub type __clockid_t = c_int;
pub type __timer_t = *mut c_void;
pub type __blksize_t = c_long;
pub type __blkcnt_t = c_long;
pub type __blkcnt64_t = c_long;
pub type __fsblkcnt_t = c_ulong;
pub type __fsblkcnt64_t = c_ulong;
pub type __fsfilcnt_t = c_ulong;
pub type __fsfilcnt64_t = c_ulong;
pub type __fsword_t = c_long;
pub type __ssize_t = c_long;
pub type __syscall_slong_t = c_long;
pub type __syscall_ulong_t = c_ulong;
pub type __loff_t = __off64_t;
pub type __caddr_t = *mut c_char;
pub type __intptr_t = c_long;
pub type __socklen_t = c_uint;
pub type __sig_atomic_t = c_int;
pub type int_least8_t = __int_least8_t;
pub type int_least16_t = __int_least16_t;
pub type int_least32_t = __int_least32_t;
pub type int_least64_t = __int_least64_t;
pub type uint_least8_t = __uint_least8_t;
pub type uint_least16_t = __uint_least16_t;
pub type uint_least32_t = __uint_least32_t;
pub type uint_least64_t = __uint_least64_t;
pub type int_fast8_t = c_schar;
pub type int_fast16_t = c_long;
pub type int_fast32_t = c_long;
pub type int_fast64_t = c_long;
pub type uint_fast8_t = c_uchar;
pub type uint_fast16_t = c_ulong;
pub type uint_fast32_t = c_ulong;
pub type uint_fast64_t = c_ulong;
pub type intmax_t = __intmax_t;
pub type uintmax_t = __uintmax_t;
#[repr(C)]
#[derive(Copy, Clone)]
pub union Axis3i16 {
    pub __bindgen_anon_1: Axis3i16__bindgen_ty_1,
    pub axis: [i16; 3usize],
}
#[repr(C)]
#[derive(Debug, Copy, Clone)]
pub struct Axis3i16__bindgen_ty_1 {
    pub x: i16,
    pub y: i16,
    pub z: i16,
}
#[repr(C)]
#[derive(Copy, Clone)]
pub union Axis3i32 {
    pub __bindgen_anon_1: Axis3i32__bindgen_ty_1,
    pub axis: [i32; 3usize],
}
#[repr(C)]
#[derive(Debug, Copy, Clone)]
pub struct Axis3i32__bindgen_ty_1 {
    pub x: i32,
    pub y: i32,
    pub z: i32,
}
#[repr(C)]
#[derive(Copy, Clone)]
pub union Axis3i64 {
    pub __bindgen_anon_1: Axis3i64__bindgen_ty_1,
    pub axis: [i64; 3usize],
}
#[repr(C)]
#[derive(Debug, Copy, Clone)]
pub struct Axis3i64__bindgen_ty_1 {
    pub x: i64,
    pub y: i64,
    pub z: i64,
}
#[repr(C)]
#[derive(Copy, Clone)]
pub union Axis3f {
    pub __bindgen_anon_1: Axis3f__bindgen_ty_1,
    pub axis: [f32; 3usize],
}
#[repr(C)]
#[derive(Debug, Copy, Clone)]
pub struct Axis3f__bindgen_ty_1 {
    pub x: f32,
    pub y: f32,
    pub z: f32,
}
#[repr(C, packed)]
#[derive(Debug, Copy, Clone)]
pub struct lighthouseCalibrationSweep_t {
    pub phase: f32,
    pub tilt: f32,
    pub curve: f32,
    pub gibmag: f32,
    pub gibphase: f32,
    pub ogeemag: f32,
    pub ogeephase: f32,
}
#[repr(C, packed)]
#[derive(Debug, Copy, Clone)]
pub struct lighthouseCalibration_t {
    pub sweep: [lighthouseCalibrationSweep_t; 2usize],
    pub uid: u32,
    pub valid: bool,
}
/*
#[doc = " @brief Generic function pointer type for a calibration measurement model.\n        Predict the measured sweep angle based on a position for a lighthouse rotor. The position is relative to the rotor reference frame.\n @param x meters\n @param y meters\n @param z meters\n @param t Tilt of the light plane in radians\n @param calib Calibration data for the rotor\n @return float The predicted uncompensated sweep angle of the rotor\n"]
pub type lighthouseCalibrationMeasurementModel_t = ::std::option::Option<
    unsafe extern "C" fn(
        x: f32,
        y: f32,
        z: f32,
        t: f32,
        calib: *const lighthouseCalibrationSweep_t,
    ) -> f32,
>;
*/
pub type stabilizerStep_t = u32;
#[doc = " Attitude in euler angle form"]
#[repr(C)]
#[derive(Debug, Copy, Clone)]
pub struct attitude_s {
    pub timestamp: u32,
    pub roll: f32,
    pub pitch: f32,
    pub yaw: f32,
}
#[doc = " Attitude in euler angle form"]
pub type attitude_t = attitude_s;
pub type vec3d = [f32; 3usize];
pub type mat3d = [[f32; 3usize]; 3usize];
#[repr(C)]
#[derive(Debug, Copy, Clone)]
pub struct vec3_s {
    pub timestamp: u32,
    pub x: f32,
    pub y: f32,
    pub z: f32,
}
pub type vector_t = vec3_s;
pub type point_t = vec3_s;
pub type velocity_t = vec3_s;
pub type acc_t = vec3_s;
pub type jerk_t = vec3_s;
#[repr(C)]
#[derive(Copy, Clone)]
pub struct quaternion_s {
    pub __bindgen_anon_1: quaternion_s__bindgen_ty_1,
}
#[repr(C)]
#[derive(Copy, Clone)]
pub union quaternion_s__bindgen_ty_1 {
    pub __bindgen_anon_1: quaternion_s__bindgen_ty_1__bindgen_ty_1,
    pub __bindgen_anon_2: quaternion_s__bindgen_ty_1__bindgen_ty_2,
}
#[repr(C)]
#[derive(Debug, Copy, Clone)]
pub struct quaternion_s__bindgen_ty_1__bindgen_ty_1 {
    pub q0: f32,
    pub q1: f32,
    pub q2: f32,
    pub q3: f32,
}
#[repr(C)]
#[derive(Debug, Copy, Clone)]
pub struct quaternion_s__bindgen_ty_1__bindgen_ty_2 {
    pub x: f32,
    pub y: f32,
    pub z: f32,
    pub w: f32,
}
pub type quaternion_t = quaternion_s;
pub const measurementSource_t_MeasurementSourceLocationService: measurementSource_t = 0;
pub const measurementSource_t_MeasurementSourceLighthouse: measurementSource_t = 0;
pub type measurementSource_t = u32;
#[repr(C)]
#[derive(Copy, Clone)]
pub struct tdoaMeasurement_s {
    pub __bindgen_anon_1: tdoaMeasurement_s__bindgen_ty_1,
    pub __bindgen_anon_2: tdoaMeasurement_s__bindgen_ty_2,
    pub distanceDiff: f32,
    pub stdDev: f32,
}
#[repr(C)]
#[derive(Copy, Clone)]
pub union tdoaMeasurement_s__bindgen_ty_1 {
    pub anchorPositions: [point_t; 2usize],
    pub __bindgen_anon_1: tdoaMeasurement_s__bindgen_ty_1__bindgen_ty_1,
}
#[repr(C)]
#[derive(Debug, Copy, Clone)]
pub struct tdoaMeasurement_s__bindgen_ty_1__bindgen_ty_1 {
    pub anchorPositionA: point_t,
    pub anchorPositionB: point_t,
}
#[repr(C)]
#[derive(Copy, Clone)]
pub union tdoaMeasurement_s__bindgen_ty_2 {
    pub anchorIds: [u8; 2usize],
    pub __bindgen_anon_1: tdoaMeasurement_s__bindgen_ty_2__bindgen_ty_1,
}
#[repr(C)]
#[derive(Debug, Copy, Clone)]
pub struct tdoaMeasurement_s__bindgen_ty_2__bindgen_ty_1 {
    pub anchorIdA: u8,
    pub anchorIdB: u8,
}
pub type tdoaMeasurement_t = tdoaMeasurement_s;
#[repr(C)]
#[derive(Debug, Copy, Clone)]
pub struct baro_s {
    pub pressure: f32,
    pub temperature: f32,
    pub asl: f32,
}
pub type baro_t = baro_s;
#[repr(C)]
#[derive(Copy, Clone)]
pub struct positionMeasurement_s {
    pub __bindgen_anon_1: positionMeasurement_s__bindgen_ty_1,
    pub stdDev: f32,
    pub source: measurementSource_t,
}
#[repr(C)]
#[derive(Copy, Clone)]
pub union positionMeasurement_s__bindgen_ty_1 {
    pub __bindgen_anon_1: positionMeasurement_s__bindgen_ty_1__bindgen_ty_1,
    pub pos: [f32; 3usize],
}
#[repr(C)]
#[derive(Debug, Copy, Clone)]
pub struct positionMeasurement_s__bindgen_ty_1__bindgen_ty_1 {
    pub x: f32,
    pub y: f32,
    pub z: f32,
}
pub type positionMeasurement_t = positionMeasurement_s;
#[repr(C)]
#[derive(Copy, Clone)]
pub struct poseMeasurement_s {
    pub __bindgen_anon_1: poseMeasurement_s__bindgen_ty_1,
    pub quat: quaternion_t,
    pub stdDevPos: f32,
    pub stdDevQuat: f32,
}
#[repr(C)]
#[derive(Copy, Clone)]
pub union poseMeasurement_s__bindgen_ty_1 {
    pub __bindgen_anon_1: poseMeasurement_s__bindgen_ty_1__bindgen_ty_1,
    pub pos: [f32; 3usize],
}
#[repr(C)]
#[derive(Debug, Copy, Clone)]
pub struct poseMeasurement_s__bindgen_ty_1__bindgen_ty_1 {
    pub x: f32,
    pub y: f32,
    pub z: f32,
}
pub type poseMeasurement_t = poseMeasurement_s;
#[repr(C)]
#[derive(Copy, Clone)]
pub struct distanceMeasurement_s {
    pub __bindgen_anon_1: distanceMeasurement_s__bindgen_ty_1,
    pub anchorId: u8,
    pub distance: f32,
    pub stdDev: f32,
}
#[repr(C)]
#[derive(Copy, Clone)]
pub union distanceMeasurement_s__bindgen_ty_1 {
    pub __bindgen_anon_1: distanceMeasurement_s__bindgen_ty_1__bindgen_ty_1,
    pub pos: [f32; 3usize],
}
#[repr(C)]
#[derive(Debug, Copy, Clone)]
pub struct distanceMeasurement_s__bindgen_ty_1__bindgen_ty_1 {
    pub x: f32,
    pub y: f32,
    pub z: f32,
}
pub type distanceMeasurement_t = distanceMeasurement_s;
#[repr(C)]
#[derive(Debug, Copy, Clone)]
pub struct zDistance_s {
    pub timestamp: u32,
    pub distance: f32,
}
pub type zDistance_t = zDistance_s;
#[repr(C)]
#[derive(Copy, Clone)]
pub struct sensorData_s {
    pub acc: Axis3f,
    pub gyro: Axis3f,
    pub mag: Axis3f,
    pub baro: baro_t,
    pub interruptTimestamp: u64,
}
pub type sensorData_t = sensorData_s;
#[repr(C)]
#[derive(Copy, Clone)]
pub struct state_s {
    pub attitude: attitude_t,
    pub attitudeQuaternion: quaternion_t,
    pub position: point_t,
    pub velocity: velocity_t,
    pub acc: acc_t,
}
pub type state_t = state_s;
pub const control_mode_e_controlModeLegacy: control_mode_e = 0;
pub const control_mode_e_controlModeForceTorque: control_mode_e = 1;
pub const control_mode_e_controlModeForce: control_mode_e = 2;
pub type control_mode_e = u32;
pub use self::control_mode_e as control_mode_t;
#[repr(C)]
#[derive(Copy, Clone)]
pub struct control_s {
    pub __bindgen_anon_1: control_s__bindgen_ty_1,
    pub controlMode: control_mode_t,
}
#[repr(C)]
#[derive(Copy, Clone)]
pub union control_s__bindgen_ty_1 {
    pub __bindgen_anon_1: control_s__bindgen_ty_1__bindgen_ty_1,
    pub __bindgen_anon_2: control_s__bindgen_ty_1__bindgen_ty_2,
    pub normalizedForces: [f32; 4usize],
}
#[repr(C)]
#[derive(Debug, Copy, Clone)]
pub struct control_s__bindgen_ty_1__bindgen_ty_1 {
    pub roll: i16,
    pub pitch: i16,
    pub yaw: i16,
    pub thrust: f32,
}
#[repr(C)]
#[derive(Copy, Clone)]
pub struct control_s__bindgen_ty_1__bindgen_ty_2 {
    pub thrustSi: f32,
    pub __bindgen_anon_1: control_s__bindgen_ty_1__bindgen_ty_2__bindgen_ty_1,
}
#[repr(C)]
#[derive(Copy, Clone)]
pub union control_s__bindgen_ty_1__bindgen_ty_2__bindgen_ty_1 {
    pub torque: [f32; 3usize],
    pub __bindgen_anon_1: control_s__bindgen_ty_1__bindgen_ty_2__bindgen_ty_1__bindgen_ty_1,
}
#[repr(C)]
#[derive(Debug, Copy, Clone)]
pub struct control_s__bindgen_ty_1__bindgen_ty_2__bindgen_ty_1__bindgen_ty_1 {
    pub torqueX: f32,
    pub torqueY: f32,
    pub torqueZ: f32,
}
pub type control_t = control_s;
#[repr(C)]
#[derive(Copy, Clone)]
pub union motors_thrust_uncapped_t {
    pub list: [i32; 4usize],
    pub motors: motors_thrust_uncapped_t__bindgen_ty_1,
}
#[repr(C)]
#[derive(Debug, Copy, Clone)]
pub struct motors_thrust_uncapped_t__bindgen_ty_1 {
    pub m1: i32,
    pub m2: i32,
    pub m3: i32,
    pub m4: i32,
}
#[repr(C)]
#[derive(Copy, Clone)]
pub union motors_thrust_pwm_t {
    pub list: [u16; 4usize],
    pub motors: motors_thrust_pwm_t__bindgen_ty_1,
}
#[repr(C)]
#[derive(Debug, Copy, Clone)]
pub struct motors_thrust_pwm_t__bindgen_ty_1 {
    pub m1: u16,
    pub m2: u16,
    pub m3: u16,
    pub m4: u16,
}
pub const mode_e_modeDisable: mode_e = 0;
pub const mode_e_modeAbs: mode_e = 1;
pub const mode_e_modeVelocity: mode_e = 2;
pub type mode_e = u32;
pub use self::mode_e as stab_mode_t;
#[repr(C)]
#[derive(Copy, Clone)]
pub struct setpoint_s {
    pub timestamp: u32,
    pub attitude: attitude_t,
    pub attitudeRate: attitude_t,
    pub attitudeQuaternion: quaternion_t,
    pub thrust: f32,
    pub position: point_t,
    pub velocity: velocity_t,
    pub acceleration: acc_t,
    pub jerk: jerk_t,
    pub velocity_body: bool,
    pub mode: setpoint_s__bindgen_ty_1,
}
#[repr(C)]
#[derive(Debug, Copy, Clone)]
pub struct setpoint_s__bindgen_ty_1 {
    pub x: stab_mode_t,
    pub y: stab_mode_t,
    pub z: stab_mode_t,
    pub roll: stab_mode_t,
    pub pitch: stab_mode_t,
    pub yaw: stab_mode_t,
    pub quat: stab_mode_t,
}
pub type setpoint_t = setpoint_s;
#[doc = " Estimate of position"]
#[repr(C)]
#[derive(Debug, Copy, Clone)]
pub struct estimate_s {
    pub timestamp: u32,
    pub position: point_t,
}
#[doc = " Estimate of position"]
pub type estimate_t = estimate_s;
#[doc = " Setpoint for althold"]
#[repr(C)]
#[derive(Debug, Copy, Clone)]
pub struct setpointZ_s {
    pub z: f32,
    pub isUpdate: bool,
}
#[doc = " Setpoint for althold"]
pub type setpointZ_t = setpointZ_s;
#[doc = " Flow measurement"]
#[repr(C)]
#[derive(Copy, Clone)]
pub struct flowMeasurement_s {
    pub timestamp: u32,
    pub __bindgen_anon_1: flowMeasurement_s__bindgen_ty_1,
    pub stdDevX: f32,
    pub stdDevY: f32,
    pub dt: f32,
}
#[repr(C)]
#[derive(Copy, Clone)]
pub union flowMeasurement_s__bindgen_ty_1 {
    pub __bindgen_anon_1: flowMeasurement_s__bindgen_ty_1__bindgen_ty_1,
    pub dpixel: [f32; 2usize],
}
#[repr(C)]
#[derive(Debug, Copy, Clone)]
pub struct flowMeasurement_s__bindgen_ty_1__bindgen_ty_1 {
    pub dpixelx: f32,
    pub dpixely: f32,
}
#[doc = " Flow measurement"]
pub type flowMeasurement_t = flowMeasurement_s;
#[doc = " TOF measurement"]
#[repr(C)]
#[derive(Debug, Copy, Clone)]
pub struct tofMeasurement_s {
    pub timestamp: u32,
    pub distance: f32,
    pub stdDev: f32,
}
#[doc = " TOF measurement"]
pub type tofMeasurement_t = tofMeasurement_s;
#[doc = " Absolute height measurement"]
#[repr(C)]
#[derive(Debug, Copy, Clone)]
pub struct heightMeasurement_s {
    pub timestamp: u32,
    pub height: f32,
    pub stdDev: f32,
}
#[doc = " Absolute height measurement"]
pub type heightMeasurement_t = heightMeasurement_s;
#[doc = " Yaw error measurement"]
#[repr(C)]
#[derive(Debug, Copy, Clone)]
pub struct yawErrorMeasurement_t {
    pub timestamp: u32,
    pub yawError: f32,
    pub stdDev: f32,
}
#[doc = " Sweep angle measurement"]
#[repr(C)]
#[derive(Debug, Copy, Clone)]
pub struct sweepAngleMeasurement_t {
    pub timestamp: u32,
    pub sensorPos: *const vec3d,
    pub rotorPos: *const vec3d,
    pub rotorRot: *const mat3d,
    pub rotorRotInv: *const mat3d,
    pub sensorId: u8,
    pub baseStationId: u8,
    pub sweepId: u8,
    pub t: f32,
    pub measuredSweepAngle: f32,
    pub stdDev: f32,
}
#[doc = " gyroscope measurement"]
#[repr(C)]
#[derive(Copy, Clone)]
pub struct gyroscopeMeasurement_t {
    pub gyro: Axis3f,
}
#[doc = " accelerometer measurement"]
#[repr(C)]
#[derive(Copy, Clone)]
pub struct accelerationMeasurement_t {
    pub acc: Axis3f,
}
#[doc = " barometer measurement"]
#[repr(C)]
#[derive(Debug, Copy, Clone)]
pub struct barometerMeasurement_t {
    pub baro: baro_t,
}
pub const StateEstimatorType_StateEstimatorTypeAutoSelect: StateEstimatorType = 0;
pub const StateEstimatorType_StateEstimatorTypeComplementary: StateEstimatorType = 1;
pub const StateEstimatorType_StateEstimatorTypeKalman: StateEstimatorType = 2;
pub const StateEstimatorType_StateEstimatorType_COUNT: StateEstimatorType = 3;
pub type StateEstimatorType = u32;
pub const MeasurementType_MeasurementTypeTDOA: MeasurementType = 0;
pub const MeasurementType_MeasurementTypePosition: MeasurementType = 1;
pub const MeasurementType_MeasurementTypePose: MeasurementType = 2;
pub const MeasurementType_MeasurementTypeDistance: MeasurementType = 3;
pub const MeasurementType_MeasurementTypeTOF: MeasurementType = 4;
pub const MeasurementType_MeasurementTypeAbsoluteHeight: MeasurementType = 5;
pub const MeasurementType_MeasurementTypeFlow: MeasurementType = 6;
pub const MeasurementType_MeasurementTypeYawError: MeasurementType = 7;
pub const MeasurementType_MeasurementTypeSweepAngle: MeasurementType = 8;
pub const MeasurementType_MeasurementTypeGyroscope: MeasurementType = 9;
pub const MeasurementType_MeasurementTypeAcceleration: MeasurementType = 10;
pub const MeasurementType_MeasurementTypeBarometer: MeasurementType = 11;
pub type MeasurementType = u32;
#[repr(C)]
#[derive(Copy, Clone)]
pub struct measurement_t {
    pub type_: MeasurementType,
    pub data: measurement_t__bindgen_ty_1,
}
#[repr(C)]
#[derive(Copy, Clone)]
pub union measurement_t__bindgen_ty_1 {
    pub tdoa: tdoaMeasurement_t,
    pub position: positionMeasurement_t,
    pub pose: poseMeasurement_t,
    pub distance: distanceMeasurement_t,
    pub tof: tofMeasurement_t,
    pub height: heightMeasurement_t,
    pub flow: flowMeasurement_t,
    pub yawError: yawErrorMeasurement_t,
    pub sweepAngle: sweepAngleMeasurement_t,
    pub gyroscope: gyroscopeMeasurement_t,
    pub acceleration: accelerationMeasurement_t,
    pub barometer: barometerMeasurement_t,
}
extern "C" {
    pub fn stateEstimatorInit(estimator: StateEstimatorType);
}
extern "C" {
    pub fn stateEstimatorTest() -> bool;
}
extern "C" {
    pub fn stateEstimatorSwitchTo(estimator: StateEstimatorType);
}
extern "C" {
    pub fn stateEstimator(state: *mut state_t, stabilizerStep: stabilizerStep_t);
}
extern "C" {
    pub fn stateEstimatorGetType() -> StateEstimatorType;
}
extern "C" {
    pub fn stateEstimatorGetName() -> *const c_char;
}
extern "C" {
    pub fn estimatorEnqueue(measurement: *const measurement_t);
}
extern "C" {
    pub fn estimatorDequeue(measurement: *mut measurement_t) -> bool;
}

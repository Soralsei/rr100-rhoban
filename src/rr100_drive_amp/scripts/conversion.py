import numpy as np
from geometry_msgs.msg import *

stamped_type_to_attr = {
    AccelStamped: 'accel',
    AccelWithCovarianceStamped: 'accel',
    InertiaStamped: 'inertia',
    PointStamped: 'point',
    PolygonStamped: 'polygon',
    PoseStamped: 'pose',
    PoseWithCovariance: 'pose',
    QuaternionStamped: 'quaternion',
    TransformStamped: 'transform',
    TwistStamped: 'twist',
    TwistWithCovarianceStamped: 'twist',
    Vector3Stamped: 'vector',
    WrenchStamped: 'wrench',
}

# def _assert_has_shape(array, *shapes):
#     """Raises a ValueError if `array` cannot be reshaped into any of
#     `shapes`."""

#     # Assumes that shapes are tuples and sequences of shapes are lists
#     if array.shape not in shapes:
#         raise ValueError(
#             f'Expected array of shape(s): {shapes}, received {array.shape}.'
#         )

def cast_to_dtype(array, dtype):
    """Raises a TypeError if `array` cannot be casted to `dtype` without
    loss of precision."""

    min_dtype = np.min_scalar_type(array)

    if not np.can_cast(min_dtype, dtype):
        raise TypeError(f'Cannot safely cast array {array} to dtype {dtype}.')

    return array.astype(dtype)

# def to_message(message_type, *args, **kwargs):
#     """Converts a NumPy representation into the specified ROS message type."""
    
#     return message_type(message_type, *args, **kwargs)

def unstamp(message):
    attribute = stamped_type_to_attr.get(message.__class__)    
    if attribute is not None:
        message = getattr(message, attribute)
        
    return message

def kinematics_to_numpy(message):
    message = unstamp(message)
    is_wrench = isinstance(message, Wrench)
    
    linear = message.force if is_wrench else message.linear
    angular = message.torque if is_wrench else message.angular
    
    return vector_to_numpy(linear), vector_to_numpy(angular)

def kinematics_with_covariance_to_numpy(message):
    message = unstamp(message)
    is_accel = isinstance(message, AccelWithCovarianceStamped)
    
    kinematics = message.accel if is_accel else message.twist
    
    linear, angular = kinematics_to_numpy(kinematics)
    covariance = np.array(message.covariance, dtype=np.float64).reshape(6, 6)
    
    return linear, angular, covariance

def numpy_to_vector(message_type, array):

    dtype = np.float32 if message_type is Point32 else np.float64
    array = cast_to_dtype(array, dtype)

    return message_type(*array[:3])

def vector_to_numpy(vec):
    '''
    Converts a ros vector type to a numpy array
    '''
    vec = unstamp(vec)
    data = [vec.x, vec.y, vec.z]
    d_type = np.float32 if isinstance(vec, Point32) else np.float64
    
    return np.array(data, dtype=d_type)

def numpy_to_kinematics(message_type, linear, angular):
    is_wrench = message_type is Wrench

    linear_key = 'force' if is_wrench else 'linear'
    angular_key = 'torque' if is_wrench else 'angular'

    kwargs = {
        linear_key: numpy_to_vector(Vector3, linear),
        angular_key: numpy_to_vector(Vector3, angular)
    }

    return message_type(**kwargs)

def numpy_to_kinematics_with_covariance(message_type, linear, angular, covariance):
    is_wrench = message_type is Wrench

    linear_key = 'force' if is_wrench else 'linear'
    angular_key = 'torque' if is_wrench else 'angular'

    kwargs = {
        linear_key: numpy_to_vector(Vector3, linear),
        angular_key: numpy_to_vector(Vector3, angular)
    }

    return message_type(**kwargs)
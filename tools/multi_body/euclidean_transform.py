import numpy as np


def translate(v: np.array, *args, **kwargs) -> np.array:
    """
    Translate vector/matrix v, see translation_matrix for kwargs.
    """
    p = np.array(v)
    p = _add_ones_to_last_row(p)

    t = translation_matrix(*args, **kwargs)

    p_t = t @ p
    v_t = p_t[:-1]

    return v_t


def _add_ones_to_last_row(p):
    if p.ndim > 1:
        ones = np.ones((1, p.shape[1]))
        return np.vstack((p, ones))
    else:
        return np.append(p, 1)


def scale(v: np.array, *args, **kwargs) -> np.array:
    """
    Scale vector/matrix v, see scaling_matrix for kwargs.
    """
    s = scaling_matrix(*args, **kwargs)
    v_s = s @ v

    return v_s


def rotate(v: np.array, *args, **kwargs) -> np.array:
    """
    Rotate vector/matrix v, see rotation_matrix for kwargs.
    """
    r = rotation_matrix(*args, **kwargs)
    v_r = r @ v

    return v_r


def translation_matrix(x_t: float = 0, y_t: float = 0, z_t: float = 0) -> np.array:
    """
    Returns the translates matrix, given (x_t, y_t, z_t).
    """
    return np.array([[1, 0, 0, x_t],
                     [0, 1, 0, y_t],
                     [0, 0, 1, z_t],
                     [0, 0, 0, 1]])


def scaling_matrix(x_s: float = 0, y_s: float = 0, z_s: float = 0) -> np.array:
    """
    Returns the scaling matrix, given (x_s, y_s, z_s).
    """
    return np.array([[x_s, 0, 0],
                     [0, y_s, 0],
                     [0, 0, z_s]])


def rotation_matrix(x_rad: float = 0, y_rad: float = 0, z_rad: float = 0) -> np.array:
    """
    Returns the rotation matrix x-y-z (phi-theta-psi),
    give (x_rad, y_rad, z_rad)
    """
    r_x = np.array([[1, 0, 0],
                    [0, np.cos(x_rad), -np.sin(x_rad)],
                    [0, np.sin(x_rad), np.cos(x_rad)]])

    r_y = np.array([[np.cos(y_rad), 0, np.sin(y_rad)],
                    [0, 1, 0],
                    [-np.sin(y_rad), 0, np.cos(y_rad)]])

    r_z = np.array([[np.cos(z_rad), -np.sin(z_rad), 0],
                    [np.sin(z_rad), np.cos(z_rad), 0],
                    [0, 0, 1]])

    r = r_z @ r_y @ r_x
    return r

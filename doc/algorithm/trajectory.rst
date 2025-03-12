.. py:currentmodule:: lsst.ts.mthexapod

.. _lsst.ts.mthexapod.trajectory:

###############
Trajectory
###############

The main control algorithm of the hexapod is in the Simulink model: `ts_mt_hexRot_simulink <https://github.com/lsst-ts/ts_mt_hexRot_simulink>`_.
The trajectory generator is in the block of **Calculate Synchronous Accel Cmd**.
This calculates the synchronous and asynchronous movements based on the parameter: **ControlCommands_cmdSynchronousMove**.
If the value equals 1, the synchronous movement is applied.
If the value equals 0, the asynchronous movement is applied.
When the hexapod is doing the synchronous movement, all struts should arrive the target positions at the same time.

In this trajectory generator, the main inputs are the current strut positions and targets positions.
All struts are considered to move independently.
The calculation frequency is 200 Hz and the main output is the commanded acceleration (:math:`a_{i,\text{cmd}}`) at each time moment for ith strut.
Since the strut's movement is restricted by the velocity (:math:`v_{\max}`) and acceleration (:math:`a_{\max}`), the main consideration is to evaluate the movement will reach :math:`v_{\max}` or not.
If so, the time at the maximum velocity (:math:`t_{i,\text{vmax}}`) will be calculated.

Under this consideration, the maximum movement time is important.
Consider the hexapod's space first.
The hexapod has the current position: :math:`\vec{p}_{\text{now}}' = (r_{x}', r_{y}', r_{z}', x', y', z')^{T}`, where :math:`r_{x}'`, :math:`r_{y}'`, and :math:`r_{z}'` are the rotation angles around :math:`x'`, :math:`y'`, and :math:`z'` axes.
If the commanded position is :math:`\vec{p}_{\text{cmd}}'`, we have the displacement (:math:`\vec{d}'`):

.. math::
    \vec{d}' = \vec{p}_{\text{cmd}}' - \vec{p}_{\text{now}}' = ( \Delta r_{x}', \Delta r_{y}', \Delta r_{z}', \Delta x', \Delta y', \Delta z' )^{T}

Since the initial/final acceleration and velocity will be zero in the movement, the trajectory profile should be symmetric.
Therefore, only the consideration of the first half profile is required.
The absolute displacement for the radial, axial, radial angular, and axial angular directions are:

.. math::
    \begin{aligned}
    d_{\text{radial}}' &= \frac{\sqrt{\Delta x'^{2} + \Delta y'^{2}}}{2} \\
    d_{\text{axial}}' &= \frac{| \Delta z' |}{2} \\
    d_{\theta,\text{radial}}' &= \frac{\sqrt{\Delta r_{x}'^{2} + \Delta r_{y}'^{2}}}{2} \\
    d_{\theta,\text{axial}}' &= \frac{| \Delta r_{z}' |}{2}
    \end{aligned}

Assumes the maximum velocities under these four directions are: :math:`v_{\max,\text{radial}}'`, :math:`v_{\max,\text{axial}}'`, :math:`v_{\theta\max,\text{radial}}'`, and :math:`v_{\theta\max,\text{axial}}'`, and the velocity profile is a perfect triangle, we have the minimum times for each direction to be:

.. math::
    \begin{aligned}
    t_{\min,\text{radial}}' &= 2 \frac{\Delta d_{\text{radial}}'}{v_{\max,\text{radial}}'} \\
    t_{\min,\text{axial}}' &= 2 \frac{\Delta d_{\text{axial}}'}{v_{\max,\text{axial}}'} \\
    t_{\theta\min,\text{radial}}' &= 2 \frac{\Delta d_{\theta,\text{radial}}'}{v_{\theta\max,\text{radial}}'} \\
    t_{\theta\min,\text{axial}}' &= 2 \frac{\Delta d_{\theta,\text{axial}}'}{v_{\theta\max,\text{axial}}'}
    \end{aligned}

The minimum movement time (:math:`t_{\min}'`) in the hexapod space is:

.. math::
    t_{\min}' = \max(t_{\min,\text{radial}}', t_{\min,\text{axial}}', t_{\theta\min,\text{radial}}', t_{\theta\min,\text{axial}}')

Assume the struts have the current position :math:`\vec{p}_{\text{now}}` and the final target position :math:`\vec{p}_{\text{cmd}}`.
The displacement in the trajectory profile (only consider the first half profile) is:

.. math::
    \vec{d} = \frac{| \vec{p}_{\text{cmd}} - \vec{p}_{\text{now}} |}{2}

The direction to move is:

.. math::
    \vec{s} = \text{sign}(\vec{p}_{\text{cmd}} - \vec{p}_{\text{now}})

To have the ith strut's movement reaches :math:`v_{\max}`, the minimum displacement needs to be:

.. math::
    d_{\text{threshold}} = \frac{v_{\max}^2}{2a_{\max}}

If :math:`d_{i} > d_{\text{threshold}}`, the maximum movement time for strut :math:`i` in the strut's space will be:

.. math::
    t_{i,\max} = \frac{d_{i} - d_{\text{threshold}}}{v_{\max}} + \frac{v_{\max}}{a_{\max}}

Otherwise, it will be:

.. math::
    t_{i,\max} = \sqrt{\frac{2d_{i}}{a_{\max}}}

Consider the hexapod's and strut's spaces, the overall maximum movement time is:

.. math::
    T_{i,\max} = \max(t_{i,\max}, t_{\min}')

The hexapod movement is restricted by the longest struct displacement: :math:`d_{\max} = \max(\vec{d})`.
Therefore, in the synchronous movement, the overall maximum movement time is always:

.. math::
    T_{i,\max} = T_{\max} = \max(T_{i,\max}), i\in \{1..6\}

For each strut calculate the acceleration (:math:`a_{i,\text{cmd}}`), acceleration time (:math:`t_{i,\text{accel}}`), and the maximum velocity time (:math:`t_{i,\text{vmax}}`) required to reach the destination at the specific time, and try to use the minimum acceleration if possible.
The average velocity is: :math:`v_{i, \text{avg}} = d_{i} / T_{i, \max}`.
If the hexapod's movement is short and the maximum velocity will not be reached (:math:`v_{i,\text{avg}} \le v_{\max}/2`), we have:

.. math::
    \begin{aligned}
    a_{i,\text{cmd}} &= 2\frac{v_{i, \text{avg}}}{T_{i,\max}} \\
    t_{i,\text{accel}} &= T_{i,\max} \\
    t_{i,\text{vmax}} &= 0
    \end{aligned}

Otherwise, the self-consistent field method is used to solve the following equations:

.. math::
    \begin{aligned}
    t_{i,\text{accel}} &= \frac{v_{\max}}{a_{i,\text{cmd}}} \\
    T_{i,\max} &= \frac{t_{i,\text{vmax}}}{2} + t_{i,\text{accel}} \\
    v_{i,\text{avg, new}}T_{i,\max} &= \frac{1}{2}a_{i,\text{cmd}} t_{i,\text{accel}}^2 + \frac{1}{2}v_{\max}t_{i,\text{vmax}}
    \end{aligned}

:math:`v_{i,\text{avg, new}}` is the new average velocity for strut :math:`i`.
Since there are 4 variables but 3 independent equations, the boundary condition (:math:`0 \le a_{i,\text{cmd}} \le a_{\max}`) is applied to have a reasonable solution.

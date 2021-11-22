from setuptools import setup, find_packages

setup(
    name="ugglan-tools",
    packages=find_packages(),
    entry_points={
        'console_scripts': [
            'plot-data-log=data_log.plot:main',
            'plot-multi-body=multi_body.plot_drone:main',
            'plot-att-est=state_est.plot_att_est:main',
            'plot-hard-iron-offset=state_est.hard_iron_offset:main',
            'plot-motor-dyn=state_est.motor_dynamics:main',
            'plot-motor-thrust=state_est.motor_thrust:main',
            'gui-linear-sim=linear_sim.gui:main',
            'gui-6dof-sim=non_linear_sim.gui:main',
        ]
    },
    install_requires=[
        'dataclasses',
        'matplotlib',
        'numpy',
        'pandas',
        'scipy',
        'pyqtgraph',
        'PyQt5',
        'PyOpenGL',
    ],
    python_requires='>=3.7'
)

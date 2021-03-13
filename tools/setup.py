from setuptools import setup, find_packages

setup(
    name="ugglan-tools",
    packages=find_packages(),
    entry_points={
        'console_scripts': [
            'plot-data-log=data_log.plot:main',
            'plot-multi-body=multi_body.plot_drone:main',
            'plot-att-est=state_est.attitude:main',
            'plot-hard-iron-offset=state_est.hard_iron_offset:main',
            'plot-motor-dyn=state_est.motor_dynamics:main',
            'plot-motor-thrust=state_est.motor_thrust:main',
            'gui-state-ctrl=state_ctrl.gui:main',
        ]
    },
    install_requires=[
        'dataclasses',
        'matplotlib',
        'numpy',
        'scipy',
    ],
    python_requires='>=3.7'
)

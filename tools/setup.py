from setuptools import setup, find_packages

setup(
    name="ugglan-tools",
    version="0.0.1",
    packages=find_packages(),
    entry_points={
        'console_scripts': [
            'plot-data-log=data_log.plot:main',
            'plot-multi-body=multi_body_sim.plot_drone:main',
            'plot-att-est=attitude_est.plot:main',
            'plot-hard-iron-offset=misc_scripts.hard_iron_offset:main',
            'plot-motor-dynamics=misc_scripts.motor_dynamics:main',
            'plot-motor-thrust=misc_scripts.motor_thrust:main',
            'plot-motor-vibrations=misc_scripts.motor_vibrations:main',
            'gui-linear-sim=linear_sim.gui:main',
            'gui-6dof-sim=non_linear_sim.gui:main',
            'gui-filter-design=filter_design.gui:main'
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
        'inputs',
    ],
    python_requires='>=3.7'
)

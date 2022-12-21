# Ugglan
Ugglan (owl in Swedish) is a hobby drone project. Making use of the
Raspberry Pi Zero 2 W (amongst other things).

<img src="./doc/source/figures/drone_multi_body.png" height="350" />

The project serves as a learning platform for the author himself and
is quite broad, containing everything from assembler to docker. It
also contains some multi body modelling and simulation as well as
state space estimation and control.

## Setup

### Prerequisites
* python >=3.9
* docker
* make

Run the setup script in the root.

```bash
source ./setup.sh
```

## Documentation
[Sphinx](https://www.sphinx-doc.org/en/master/) is used for the generation
of the documentation.

### Build
The documentation is build using `make`.

```bash
make -C ./doc
```

The generated html documentation is found under *./doc/build*.

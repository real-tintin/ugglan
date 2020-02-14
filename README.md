# Ugglan
Ugglan (owl in Swedish) is a hobby drone project. Making use of the
Raspberry Pi Zero (amongst other things).

The project serves as a learning platform for the author himself and
is quite broad, containing everything from assembler to docker and
UI web-development.

## Documentation
[Sphinx](https://www.sphinx-doc.org/en/master/) is used for the generation
of the documentation.

### Prerequisites
* Python >=3.5 (3.7 is preferred in this project)
* Sphinx

### Build
The documentation is build using `make`.

```bash
cd ./doc/ && make
```

The generated html documentation is found under *./doc/build*.

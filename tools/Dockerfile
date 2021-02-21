FROM python:3.7.9

# Setup python env
ENV ROOT /tools
WORKDIR ${ROOT}
COPY ./requirements.txt ${ROOT}

RUN pip install --upgrade pip
RUN pip install -r "requirements.txt" # TODO: Install tools package instead.

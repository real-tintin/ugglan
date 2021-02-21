FROM python:3.7.9

# Setup sphinx env
ENV ROOT /doc
WORKDIR ${ROOT}
COPY ./requirements.txt ${ROOT}

RUN pip install --upgrade pip
RUN pip install -r "requirements.txt"

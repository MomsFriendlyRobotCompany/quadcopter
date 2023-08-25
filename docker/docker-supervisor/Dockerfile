FROM alpine:latest

# && mkdir -p /apps \
RUN apk update --no-cache \
    && apk add --no-cache \
        supervisor \
        libstdc++ \
        gcompat \
    && mkdir /etc/supervisor.d \
    && mv /etc/supervisord.conf /etc/supervisord.conf.bak

RUN touch /var/run/supervisord.sock \
    && chmod 777 /var/run/supervisord.sock

COPY supervisord.conf /etc/supervisord.conf
COPY nav.ini /etc/supervisor.d/
COPY bus.ini /etc/supervisor.d/

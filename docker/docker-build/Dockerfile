FROM alpine:latest AS base

# update OS and repos
RUN apk update \
    && apk upgrade \
    && apk add --no-cache \
        libstdc++ \
        supervisor\
    && mkdir /etc/supervisor.d \
    && mv /etc/supervisord.conf /etc/supervisord.conf.bak

# Create a builder image -------------------------------
FROM base AS builder

RUN apk add --no-cache \
        nano \
        build-base \
        g++ \
        cmake \
        git

# RUN mkdir -p /quad

# this will also copy local machines's build directory
# and cause problems since linux can't use macOS binaries
# COPY --from=base /quad /src
ADD src.tar.bz2 /
# RUN unzip cpp.zip \
#     && mv cpp src

# wipe localhost's build directory and rebuild everything
# for linux
RUN mv cpp src \
    && rm -fr /src/build \
    && mkdir -p /src/build \
    && cd /src/build \
    && cmake .. \
    && make

# Create a clean runtime image ------------------------
FROM base AS runtime

# now create the final resting place for the binaries
RUN mkdir -p /apps \
    && touch /var/run/supervisord.sock \
    && chmod 777 /var/run/supervisord.sock

# copy over the apps to run
COPY --from=builder /src/build/navigator /apps/navigator
COPY --from=builder /src/build/bus /apps/bus
COPY supervisord.conf /etc/supervisord.conf
COPY nav.ini /etc/supervisor.d/
COPY bus.ini /etc/supervisor.d/

# EXPOSE 22
# EXPOSE 80

# D: don't detach and thus not a daemon
# e: errors to stderr instead of system log
# CMD ["/bin/sh"]
FROM rust:1.85.1-slim-bullseye

# Set the working directory inside the container to /src
WORKDIR /src

# Now copy the rest of the source code to the container
COPY . .

# Install system dependencies (e.g., C++ compiler, Freetype, etc.)
RUN apt-get update && apt-get install -y \
    build-essential \
    libfreetype6-dev \
    pkg-config \
    libssl-dev \
    g++ \
    libfontconfig1-dev \
    libx11-dev \
    libxft-dev \
    libxext-dev \
    libxrender-dev \
    libxcursor-dev \
    libx11-6 \
    libxft2 \
    libxext6 \
    libxrender1 \
    libxcursor1 \
    libssl-dev \
    && rm -rf /var/lib/apt/lists/*

# Build the application (in release mode)
RUN cargo build --examples --release 

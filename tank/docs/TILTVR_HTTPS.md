# tiltVR HTTPS Setup

Modern mobile browsers can block `deviceorientation` and other IMU APIs on plain HTTP pages.
If the phone stays on `Waiting for phone IMU...`, enable the built-in HTTPS endpoint.

## Config

Add or keep the following keys in `[http]`:

```ini
https_enable=1
https_port=8443
tls_cert_file=../certs/tiltvrsvr-cert.pem
tls_key_file=../certs/tiltvrsvr-key.pem
redirect_http_to_https=1
```

The cert/key paths are resolved from the executable working directory, so the default runtime layout is:

```text
build/
certs/tiltvrsvr-cert.pem
certs/tiltvrsvr-key.pem
```

## Self-signed certificate example

```bash
mkdir -p ../certs
openssl req -x509 -nodes -newkey rsa:2048 \
  -keyout ../certs/tiltvrsvr-key.pem \
  -out ../certs/tiltvrsvr-cert.pem \
  -days 365 \
  -subj "/CN=<PI_IP>"
```

## Runtime behavior

- If HTTPS starts successfully, `http://<PI_IP>:8000/web/` redirects to `https://<PI_IP>:8443/web/` when `redirect_http_to_https=1`.
- If OpenSSL support is not compiled in, or the cert/key files are missing, the server logs a warning and falls back to plain HTTP.
- `ptz.imu_timeout_ms` now defaults to `1000` because `300ms` was too short on congested 5GHz Wi-Fi.

## Phone trust requirement

The phone must trust the self-signed certificate. If the browser does not trust the certificate, the page may not open correctly and sensor APIs can still remain blocked.

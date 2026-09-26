# Setting up the ESP32TimeServer and its clients for symmetric-key authentication

## What is symmetric-key authentication (and what is it not)

Symmetric-key authentication lets NTPv4 clients and the server verify time packets using a shared secret. This helps ensure the client is getting its time from the right server. However, it is 
not a secured protocol as it does not encrypt time traffic or hide the secret from anyone who can read the key files or firmware. Accordingly, its use is targeted for the environments where there is not a concern related to time server spoofing. It works on NTPv4 requests, but not older NTP versions. Also, within ESP32TimeServer it is not an access-control requirement: clients without a key can still receive unauthenticated time responses.  However, clients requesting authenticated time responses with a missing, outdated, or incorrect 
shared secret will be out of luck.  

## Compatibility

ESP32TimeServer supports the distinct symmetric-key NTPv4 authentication implementations of [Meinberg](https://www.meinbergglobal.com/english/sw/ntp.htm) (Windows) and [Chrony](https://chrony-project.org/download.html) (Linux).

W32Time (Windows) and macOS's built-in automated time service (`timed` on modern macOS) do not support symmetric-key NTPv4 authentication, and thus are not supported.



## Set up ESP32TimeServer

1. Set `SYMMETRIC_KEY_AUTHENTICATION_ENABLED` to `1` in `main/ESP32TimeServerSettings.h`.
2. In `main/ESP32TimeServerSettingsSecret.h` contains the symmetric-key authentication key ids and secrets in 
the form `{key_id, "key_value"}`. There must be between one and four such entries defined. If there are multiple entries, they should be unique. A `key_id` must be from 1 to 65535 and unique. `key_value` (the secret) must contain exactly 32 alphanumeric characters. Multiple clients may use the same symmetric-key authentication key ids and secrets.

1. Generate a new random 32-character alphanumeric secret for each key ID. The PowerShell script below outputs a series of characters which can be placed between the quotes in `key_value` field:

   ```powershell
   $alphabet = 'ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789'.ToCharArray()

   $randomNumberGenerator = [System.Security.Cryptography.RandomNumberGenerator]::Create()
   $characters = [char[]]::new(32)
   $randomByte = [byte[]]::new(1)
   $limit = 256 - (256 % $alphabet.Count)

   for ($index = 0; $index -lt $characters.Length; $index++) {
      do {
         $randomNumberGenerator.GetBytes($randomByte)
      } while ($randomByte[0] -ge $limit)
      $characters[$index] = $alphabet[$randomByte[0] % $alphabet.Count]
   }
   $randomNumberGenerator.Dispose()

   -join $characters
   ```

   Example output to copy and paste into `key_value` (without quotes):
   ```text
   V04ORr3XZwSzO06MUeObebAeJ3Yn0qyo
   ```

   Place the output inside the quotes of the corresponding `symmetric_keys` entry.
   Do not commit settings containing live keys to a public repository. Limit
   access to the settings header, firmware image, build output, and client key
   files to trusted administrators. The secret is recoverable from firmware and
   build materials and files on client systems, so it is not a protection against
   extraction.

2. Build and flash the selected ESP32-P4 board configuration.

3. When the server displays its 'Open for business' message in the console log,
   the client-specific key-file entry will be visible ( a few lines above the `Open 
   business` message) as `Authorization key(s) for use with Meinberg/Chrony ...`. 
   The content lines differ only in their client-file encoding. 
   Treat these console log values as shared secrets: do not publish, retain, or 
   transmit them insecurely.

The key settings are compiled into the application and are not stored in NVS. A
reset-button GNSS-state cleanup does not alter it, but flashing an image built
without that key does. Keep an encrypted, access-controlled backup outside the
repository.

### Add, rotate, and revoke keys

To add an entry, add one new entry with a unique `key_id` and `key_value`,
build, and flash. 
Existing clients can continue using their old entry/entries while the 
new key is active. Add separate keys for staged rotation or client isolation, 
not because Meinberg and chrony require different key lists.

To rotate a entry, add the replacement entry first, deploy it, update and verify
each client, then remove the old entry in a later build.
Do not change  `key_value` under an existing Key ID during a staged rotation:
that immediately breaks clients still using the old secret.

To revoke a compromised entry, remove it, rebuild, flash, and replace 
the entry on every affected client. The removal takes effect only after the newly
built firmware is running. Never publish the entries found in the source control,
logs, packet captures, etc. (for example in support requests).

## Windows: Meinberg NTP

[Meinberg's authentication instructions](https://kb.meinbergglobal.com/kb/time_sync/ntp/configuration/ntp_authentication)
use a private key file, a `keys` directive, `trustedkey`, and a `server ... key`
association. 

The following is an example of what should be added to the `ntp.conf` file usually found in the `C:\Program Files (x86)\NTP\etc` or `C:\Program Files\NTP\etc` folder of the Windows machine(s) running Meinberg:

```text
restrict 127.0.0.1
keys "C:\Program Files (x86)\NTP\etc\ntp.keys"
trustedkey 1
server 192.168.1.24 iburst key 1
```
> Note: the `restrict 127.0.0.1` entry may already be in your ntp.conf file, if so do not duplicated it.

Use the same key ID in `ntp.keys`, `trustedkey`, and `server ... key`. For the
example above, the relevant `ntp.keys` line is:

```text
 1 SHA256 5630344F527233585A77537A4F30364D55654F62656241654A33596E3071796F
```
> With the real contents copied from line(s) below "Authorization key(s) for use with Meinberg" in the ESP32 console log.

 Copy the value after `Authorization key(s) for use with Meinberg ...` from the server's console log as
into the `ntp.keys` file. **Do not copy `key_value` directly into `ntp.keys`:
the console log entry is the client format.**

Restart the Meinberg NTP Daemon after any configuration or key-file change.

To confirm everything is working as expected, use the Windows Command Line command:

```
"C:\Program Files (x86)\NTP\bin\ntpq" -4 -c as 127.0.0.1
```
and you should see results simular to this:

<div style="margin-left: 40px">
<pre><code>
ind assid status  conf reach auth condition  last_event cnt
===========================================================
  1 36257  f61a   yes   yes   ok   sys.peer    sys_peer  1
</code></pre>
</div>

The [reference `ntpd` 4.2.8 authentication documentation](https://www.ntp.org/documentation/4.2.8-series/authentic/)
describes the matching keyed-digest construction: SHA256 over the secret
concatenated with NTP packet fields, excluding the MAC. The firmware uses the
first 20 digest bytes so its NTPv4 trailer is 24 bytes. Record the exact
Meinberg installer and `ntpd` build, capture no secrets, and verify both an
authenticated request at the server and `auth=ok` at the client before claiming
support. For *unauthenticated* time only, use the existing
[Windows setup instructions](../Setup.md#windows--meinberg-ntp-daemon-recommended).

## Linux: chrony

Chrony's [configuration manual](https://chrony-project.org/doc/latest/chrony.conf.html)
documents a `key` option for a `server` and a `keyfile` containing numbered
keys. 

Here is an example of what the `etc/chrony/chrony.conf` file should contain:

```text
keyfile /etc/chrony/chrony.keys
server 192.0.2.10 key 1 version 4 iburst
authselectmode require
```
Here too is an example of what the `etc/chrony/chrony.keys` file should contain:
```
 1 SHA256 HEX:5630344F527233585A77537A4F30364D55654F62656241654A33596E3071796F
```
> With the real contents copied from line(s) below "Authorization key(s) for use with Chrony" in the ESP32 console log.
>
Copy the value after Authorization key(s) for use with Chony ... from the server's console log into the chrony.keys file. Do not copy key_value directly into ntp.keys: the console log entry is the client format.


Restart the appropriate
`chrony`/`chronyd` service after editing the configuration
```bash
sudo systemctl restart chronyd
```

consult
`chronyc sources -v`,
`chronyc tracking`, and, if supported, `chronyc authdata` for diagnostics.

The [chrony NTP authentication code](https://raw.githubusercontent.com/mlichvar/chrony/master/ntp_auth.c)
places the Key ID before the digest while its hash implementation calculates
`SHA256(secret || packet)`, matching this firmware. Chrony truncates a NTPv4
SHA256 digest to fit the 24-byte MAC trailer. `authselectmode require` controls
source selection; it cannot repair an incorrect key, Key ID, or packet profile.
Record the exact package version and crypto backend, then verify a signed request
and reply before claiming support. For unauthenticated time, use the existing
[Linux setup instructions](../Setup.md#linux) instead.


To confirm everything is working as expected, use these two Terminal Line commands:

```bash
sudo chronyc ntpdata 192.168.1.24
```
(where you replace 192.168.1.24 with the IPv4 address of your ESP32TimeServer)

You should see an output like this:
<div style="margin-left: 40px">
<pre><code>
Remote address  : 192.168.1.24 (C0A80118)
Remote port     : 123
Local address   : 192.168.1.30 (C0A8011E)
Leap status     : Normal
Version         : 4
Mode            : Server
Stratum         : 1
Poll interval   : 4 (16 seconds)
Precision       : -13 (0.000122070 seconds)
Root delay      : 0.000000 seconds
Root dispersion : 0.001007 seconds
Reference ID    : 47505300 (GPS)
Reference time  : Fri Sep 25 12:05:32 2026
Offset          : -0.000062175 seconds
Peer delay      : 0.000191435 seconds
Peer dispersion : 0.000122104 seconds
Response time   : 0.000326989 seconds
Jitter asymmetry: +0.00
NTP tests       : 111 111 1111
Interleaved     : No
Authenticated   : Yes
TX timestamping : Kernel
RX timestamping : Kernel
Total TX        : 7
Total RX        : 7
Total valid RX  : 7
Total good RX   : 7
</code></pre>
</div>
and then
```bash
sudo chronyc authdata
```
You should see an output like this:
<div style="margin-left: 40px">
<pre><code>
Name/IP address             Mode KeyID Type KLen Last Atmp  NAK Cook CLen
=========================================================================
192.168.1.24                  SK     1    3  256    -    0    0    0    0
</code></pre>
</div>


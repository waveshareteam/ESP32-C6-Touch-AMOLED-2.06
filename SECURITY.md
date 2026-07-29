# Security policy

## Supported source

Security fixes are applied to the default branch. Historical factory images and old
workflow artifacts are not independently maintained as security release lines.

## Reporting a vulnerability

Do not open a public issue for a suspected vulnerability. Use GitHub private
vulnerability reporting for this repository when it is available. Otherwise contact
Waveshare through the official support channel linked from the product wiki and make
it clear that the report contains security-sensitive details.

Include the affected example or component, source revision, impact, reproduction
steps, and a minimal proof of concept. Do not include real credentials or personal
data.

Allow maintainers reasonable time to investigate and coordinate a fix before public
disclosure. You may be asked to validate a candidate fix on affected hardware.

## Secrets

Example code, CI logs, firmware manifests, issues, and pull requests must not contain
Wi-Fi passwords, access tokens, private keys, personal data, device provisioning
secrets, or local-machine information.

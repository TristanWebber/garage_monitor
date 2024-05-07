/**********INSTRUCTIONS**********
* This file contains dummy data. You will need to replace this with an actual root certificate.
*
* 1. Rename this file to `ca_cert.h` (it is already in .gitignore so secrets won't be commmitted to repo)
* 2. Add your own project-specific root certificate via methods described in README.md
*
*********************************/

#include <pgmspace.h>

// This should be root the CA for your broker
static const char *rootCA PROGMEM = R"EOF(
-----BEGIN CERTIFICATE-----
your_broker_certificate_goes_here
-----END CERTIFICATE-----
)EOF";

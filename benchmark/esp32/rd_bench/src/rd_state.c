/* SPDX-License-Identifier: Apache-2.0 */
/* arduino-cli copies the sketch, src/ included, into a temporary build
 * directory, so a relative path out to the library would not survive. Angle
 * brackets resolve against -I instead, which the Makefile points at the repo.
 * One shim per source keeps each its own translation unit, as on every other
 * port -- a unity build would inline across files and skew the comparison. */
#include <rd_state.c>

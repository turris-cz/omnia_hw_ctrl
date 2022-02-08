#!/bin/sh
#
# This tool builds both bootloader and application for Turris Omnia's GD32 MCU
# and stores the result into one final image, omnia-mcu-gd32-flash.bin.
#
# Bootloader is padded with 0xFF bytes to start of application.
# Application is padded with 0xFF bytes to nearest page end.

set -e

APP_START_ADDR=0x2c00
outdir=images-gd32

CROSS_COMPILE=
for c in arm-none-eabi- armv7m-softfloat-eabi-; do
	if echo | "${c}gcc" -x c -c -o /dev/null - >/dev/null 2>&1; then
		CROSS_COMPILE="${c}"
		break;
	fi
done

if [[ -z "${CROSS_COMPILE}" ]]; then
	echo "no cross compiler found" >&2
	exit 1
fi

rm -rf "${outdir}"
mkdir -p "${outdir}"

make clean && CROSS_COMPILE=${CROSS_COMPILE} make -j8 boot
chmod -x bootloader_mcu.bin
mv bootloader_mcu.bin "${outdir}/"

make clean && CROSS_COMPILE=${CROSS_COMPILE} make -j8 app
chmod -x omnia_hw_ctrl.bin
mv omnia_hw_ctrl.bin "${outdir}/"

make clean

tmp=$(mktemp)
dd if=/dev/zero ibs=1024 count=$((APP_START_ADDR/1024)) | tr "\000" "\377" >"$tmp"
dd if="${outdir}"/bootloader_mcu.bin of="$tmp" conv=notrunc
dd if="${outdir}"/omnia_hw_ctrl.bin of="$tmp" conv=notrunc oflag=append

sz=$(stat -c %s "$tmp")
dd if=/dev/zero ibs=$(((1024 - (sz % 1024)) % 1024)) count=1 | tr "\000" "\377" >>"$tmp"
sz=$(stat -c %s "$tmp")
szkb=$((sz/1024))

mv -f "$tmp" "${outdir}"/omnia-mcu-gd32-flash.bin

echo -e "\n\n\n"
echo "Generated GD32 MCU firmware for Turris Omnia"
echo "in file ${outdir}/omnia-mcu-gd32-flash.bin (size: $szkb KiB)"
echo -e "\n\n\n"

exit 0

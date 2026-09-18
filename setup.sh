#!/usr/bin/env bash
set -e

DRV_VERSION=0.0.1
DRV_IMX=imx283
SCRIPT_DIR="$(cd -- "$(dirname -- "$0")" && pwd)"
DKMS_SRC="/usr/src/${DRV_IMX}-${DRV_VERSION}"

echo "Installing ${DRV_IMX} ${DRV_VERSION} from ${SCRIPT_DIR}"

echo "Removing any existing DKMS registration"
sudo dkms remove -m "${DRV_IMX}" -v "${DRV_VERSION}" --all || true

echo "Replacing DKMS source tree"
sudo rm -rf "${DKMS_SRC}"
sudo mkdir -p "${DKMS_SRC}"
sudo cp -a "${SCRIPT_DIR}/." "${DKMS_SRC}/"

echo "Adding DKMS module"
sudo dkms add -m "${DRV_IMX}" -v "${DRV_VERSION}"

echo "Building DKMS module"
sudo dkms build -m "${DRV_IMX}" -v "${DRV_VERSION}"

echo "Installing DKMS module"
sudo dkms install -m "${DRV_IMX}" -v "${DRV_VERSION}" --force

echo "Updating module dependency database"
sudo depmod -a

echo
echo "Installed module:"
modinfo -n "${DRV_IMX}"
echo
echo "Module parameters:"
modinfo "${DRV_IMX}" | grep "^parm:" || true

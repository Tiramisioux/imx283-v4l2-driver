#!/usr/bin/bash

DRV_VERSION=0.0.2

DRV_IMX=imx283

echo "Uninstalling any previous ${DRV_IMX} module"
#dkms status ${DRV_IMX} | awk -F', ' '{print $2}' | xargs -n1 sudo dkms remove -m ${DRV_IMX} -v 
sudo dkms remove -m ${DRV_IMX} -v ${DRV_VERSION} --all

sudo mkdir -p /usr/src/${DRV_IMX}-${DRV_VERSION}

sudo cp -r $(pwd)/* /usr/src/${DRV_IMX}-${DRV_VERSION}

sudo dkms add -m ${DRV_IMX} -v ${DRV_VERSION}
sudo dkms build -m ${DRV_IMX} -v ${DRV_VERSION}
sudo dkms install -m ${DRV_IMX} -v ${DRV_VERSION} --force

# Ensure depmod prefers the DKMS copy over the in-tree IMX283 module.
sudo depmod -a "$(uname -r)"

# Fail loudly if the DKMS module was not installed where expected.
if ! test -e "/lib/modules/$(uname -r)/updates/dkms/${DRV_IMX}.ko.xz" && ! test -e "/lib/modules/$(uname -r)/updates/dkms/${DRV_IMX}.ko"; then
    echo "ERROR: DKMS did not install ${DRV_IMX} into updates/dkms" >&2
    exit 1
fi

modinfo -n ${DRV_IMX}

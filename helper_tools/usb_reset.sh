#!/usr/bin/env bash

# Run this script on OARBOTS when the devices rejects to launch with launchers.
# Very useful to prevent physically unplugging and plugging back the cables on the robots!!

# SOURCE: https://gist.github.com/planetceres/917840478e1e4d45f8373667630e51a0

# Also see the derived script: https://github.com/netinvent/usb_resetter for system level quick installation

# USB 3.1 Only
for port in $(lspci | grep xHCI | cut -d' ' -f1); do
    echo -n "0000:${port}"| sudo tee /sys/bus/pci/drivers/xhci_hcd/unbind;
    sleep 5;
    echo -n "0000:${port}" | sudo tee /sys/bus/pci/drivers/xhci_hcd/bind;
    sleep 5;
done

# All USB
for port in $(lspci | grep USB | cut -d' ' -f1); do
    echo -n "0000:${port}"| sudo tee /sys/bus/pci/drivers/xhci_hcd/unbind;
    sleep 5;
    echo -n "0000:${port}" | sudo tee /sys/bus/pci/drivers/xhci_hcd/bind;
    sleep 5;
done
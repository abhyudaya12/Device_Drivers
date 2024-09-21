#!/bin/bash

mount /dev/sdg1 /mnt

# Check if the mount was successful
if [ $? -eq 0 ]; then
    
    cp /mnt/params.yaml /root/
    
   
    umount /mnt
else
    echo "Failed to mount /dev/sdg1"
fi

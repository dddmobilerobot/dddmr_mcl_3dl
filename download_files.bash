#!/bin/bash

mkdir ~/dddmr_bags

echo -n "Do you want to download bag files (1.2GB) and pose graph (3.3MB)? (Y/N):"
read d_bag
if [ "$d_bag" != "${d_bag#[Yy]}" ] ;then 
  echo "Download bag"
  cd ~/dddmr_bags && curl -L -c cookies.txt 'https://drive.usercontent.google.com/uc?export=download&id='1NV3Jy6e3fRioA6hmc5snBQJiidK6K694 \
      | sed -rn 's/.*confirm=([0-9A-Za-z_]+).*/\1/p' > confirm.txt
  curl -L -b cookies.txt -o benanli_detention_basin_localization.zip \
      'https://drive.usercontent.google.com/download?id='1NV3Jy6e3fRioA6hmc5snBQJiidK6K694'&confirm='$(<confirm.txt)
  rm -f confirm.txt cookies.txt
  unzip benanli_detention_basin_localization.zip

  echo "Download pose graph"
  cd ~/dddmr_bags && curl -L -c cookies.txt 'https://drive.usercontent.google.com/uc?export=download&id='1NX9gEqrZVGHI1vMm2SBlqBZ3MF-rLEyJ \
      | sed -rn 's/.*confirm=([0-9A-Za-z_]+).*/\1/p' > confirm.txt
  curl -L -b cookies.txt -o benanli_detention_basin_pg.zip \
      'https://drive.usercontent.google.com/download?id='1NX9gEqrZVGHI1vMm2SBlqBZ3MF-rLEyJ'&confirm='$(<confirm.txt)
  rm -f confirm.txt cookies.txt
  unzip benanli_detention_basin_pg.zip
fi

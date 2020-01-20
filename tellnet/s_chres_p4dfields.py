#!../venv/bin/python2

import rasterio as rio
from datainterface import DataInterface
from IPython import embed
import matplotlib.pyplot as plt
import numpy as np

files_paths = [
    ("/media/seba/Samsung_2TB/TELLnet/Fields/meier-burkard/chres/"
        "20190322/chres_20190322_p4dfields_georef.tif"),
    ("/media/seba/Samsung_2TB/TELLnet/Fields/meier-burkard/chres/"
        "20190418/chres_20190418_p4dfields_georef.tif")
]

shapefile = "/media/seba/Samsung_2TB/TELLnet/Shapes/chres/chres.shp"

chres = DataInterface()

# add datasets
for file_path in files_paths:
    chres.add_dataset(file_path)

# add shapefile
chres.add_shapefile(shapefile)

# crop datasets
for dataset in chres.datasets_names:
    chres.crop_dataset(dataset)

# align datasets
chres.align_datasets('20190418')

# compute ndvi
ndvi_20190322 = chres.ndvi('20190322')
ndvi_20190418 = chres.ndvi('20190418')

# mask the field
ndvi_20190322[~chres.data_mask] = np.nan
ndvi_20190418[~chres.data_mask] = np.nan

# # plot the data
# fig, axs = plt.subplots(1, 2)
# axs[0].imshow(ndvi_20190322, cmap='RdYlGn')
# axs[0].title.set_text('20190322')
# axs[1].imshow(ndvi_20190418, cmap='RdYlGn')
# axs[1].title.set_text('20190418')
# plt.show()

# compute diff and plot it
diff = ndvi_20190418 - ndvi_20190322
plt.figure()
plt.imshow(diff, cmap='RdYlGn', vmin=-0.5, vmax=0.5)
plt.title('NDVI difference: 18.04.2019 - 22.03.2019')
plt.colorbar()
plt.savefig('NDVI_diff.png', dpi=500)

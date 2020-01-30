#!../venv/bin/python2

import rasterio as rio
from datainterface import DataInterface
from IPython import embed
import matplotlib.pyplot as plt
import numpy as np
import seaborn as sns

files_paths = [
    ("/media/seba/Samsung_2TB/TELLnet/Fields/meier-burkard/chres/"
        "20190322/chres_20190322_p4dfields_georef.tif"),
    ("/media/seba/Samsung_2TB/TELLnet/Fields/meier-burkard/chres/"
        "20190418/chres_20190418_p4dfields_georef.tif"),
    ("/media/seba/Samsung_2TB/TELLnet/Fields/meier-burkard/chres/"
        "20190507/chres_20190507_p4dfields_georef.tif")
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
chres.align_datasets('20190507')

# compute ndvi
ndvi_20190322 = chres.ndvi('20190322')
ndvi_20190418 = chres.ndvi('20190418')
ndvi_20190507 = chres.ndvi('20190507')

# mask the field
ndvi_20190322[~chres.data_mask] = np.nan
ndvi_20190418[~chres.data_mask] = np.nan
ndvi_20190507[~chres.data_mask] = np.nan

ndvis = [
    ndvi_20190322,
    ndvi_20190418,
    ndvi_20190507
]

# mask the field
for ndvi in ndvis:
    ndvi[~chres.data_mask] = np.nan

# compute diffs
diff_1 = ndvi_20190418 - ndvi_20190322
diff_2 = ndvi_20190507 - ndvi_20190418
diffs = [diff_1, diff_2]

# compute quantiles
q25 = []
q75 = []
for diff in diffs:
    q25.append(np.nanquantile(diff, 0.25))
    q75.append(np.nanquantile(diff, 0.75))
q25 = np.array(q25)
q75 = np.array(q75)

# remove NaNs for distribution plot
diffs_nan = []
for diff in diffs:
    diff = diff[~np.isnan(diff)]
    diffs_nan.append(diff)

# generate the plots
titles = [
    "18.04.2019 - 22.03.2019",
    "07.05.2019 - 18.04.2019"
]

for i, (diff, title, diff_nan) in enumerate(zip(diffs, titles, diffs_nan)):
    plt.figure()
    plt.imshow(diff, cmap='RdYlGn', vmin=0, vmax=0.06)
    plt.title('NDVI difference: {}'.format(title))
    plt.colorbar()
    plt.savefig('NDVI_diff_abs{}.png'.format(str(i+1)), dpi=500)

    plt.figure()
    plt.imshow(diff, cmap='RdYlGn', vmin=q25[i], vmax=q75[i])
    plt.title('NDVI difference: {}'.format(title))
    plt.colorbar()
    plt.savefig('NDVI_diff_rel{}.png'.format(str(i+1)), dpi=500)

    plt.figure()
    sns.distplot(diff_nan)
    plt.title('Distribution: {}'.format(title))
    plt.savefig('distr_{}.png'.format(str(i+1)), dpi=500)

#!../venv/bin/python3

import rasterio as rio
from datainterface import DataInterface
from IPython import embed
import matplotlib.pyplot as plt
import numpy as np
import seaborn as sns
import os

files_paths = [
    ("/media/seba/Samsung_2TB/Matterhorn.Project/TELLnet/Fields/meier-burkard/"
        "chres/20190322/chres_20190322_p4dfields_georef.tif"),
    ("/media/seba/Samsung_2TB/Matterhorn.Project/TELLnet/Fields/meier-burkard/"
        "chres/20190418/chres_20190418_p4dfields_georef.tif"),
    ("/media/seba/Samsung_2TB/Matterhorn.Project/TELLnet/Fields/meier-burkard/"
        "chres/20190507/chres_20190507_p4dfields_georef.tif")
]

shapefile = ("/media/seba/Samsung_2TB/Matterhorn.Project/TELLnet/Shapes/"
    "chres/chres.shp")

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
ndvis = []
for date in chres.dates:
    ndvi = chres.ndvi(date)
    ndvis.append(ndvi)

# exclude regions outside the plot / where data are missing
for i, ndvi in enumerate(ndvis):
    ndvi[~chres.data_mask] = np.nan
    ndvis[i] = ndvi

# compute diffs
diffs = []
for i in range(len(ndvis)-1):
    diff = ndvis[i+1] - ndvis[i]
    diffs.append(diff)

for i, diff in enumerate(diffs):
    diff[~chres.data_mask] = np.nan
    diffs[i] = diff

# compute quantiles
q25 = []
q50 = []
q75 = []
for diff in diffs:
    q25.append(np.nanquantile(diff, 0.25))
    q50.append(np.nanquantile(diff, 0.50))
    q75.append(np.nanquantile(diff, 0.75))
q25 = np.array(q25)
q75 = np.array(q75)
print(q50)

# generate the plots
outputs_folder = "outputs"
if not os.path.isdir(outputs_folder):
    os.mkdir(outputs_folder)

dpi = 200
for i, diff in enumerate(diffs):
    fig, ax = plt.subplots()
    plt.imshow(diff, cmap='YlGn', vmin=0, vmax=0.06)
    plt.colorbar()
    ax.axis('off')
    fname = 'diff_abs_{}.png'.format(str(i+1))
    full_fname = os.path.join(outputs_folder, fname)
    plt.savefig(full_fname, dpi=dpi, bbox_inches='tight')

    fig, ax = plt.subplots()
    plt.imshow(diff, cmap='RdYlGn', vmin=q25[i], vmax=q75[i])
    plt.colorbar()
    ax.axis('off')
    fname = 'diff_rel_{}.png'.format(str(i+1))
    full_fname = os.path.join(outputs_folder, fname)
    plt.savefig(full_fname, dpi=dpi, bbox_inches='tight')

    fig, ax = plt.subplots()
    diff = diff[~np.isnan(diff)]
    sns.distplot(diff)
    fname = 'distr_diff_{}.png'.format(str(i+1))
    full_fname = os.path.join(outputs_folder, fname)
    plt.savefig(full_fname, dpi=dpi, bbox_inches='tight')

for i, ndvi in enumerate(ndvis):
    fig, ax = plt.subplots()
    plt.imshow(ndvi, cmap='YlGn', vmin=0.75, vmax=1)
    plt.colorbar()
    ax.axis('off')
    fname = 'ndvi_{}.png'.format(chres.dates[i])
    full_fname = os.path.join(outputs_folder, fname)
    plt.savefig(full_fname, dpi=dpi, bbox_inches='tight')

    fig, ax = plt.subplots()
    ndvi = ndvi.flatten()
    ndvi = ndvi[ndvi!=np.nan]
    sns.distplot(ndvi)
    fname = 'distr_ndvi_{}.png'.format(chres.dates[i])
    full_fname = os.path.join(outputs_folder, fname)
    plt.savefig(full_fname, dpi=dpi, bbox_inches='tight')

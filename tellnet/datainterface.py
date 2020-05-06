from rasterio import MemoryFile, Affine
from rasterio.warp import Resampling
from warnings import warn
import geopandas as gpd
import numpy as np
import os
import rasterio as rio
import rasterio.mask as riom
import utils.functions as ufunc

from IPython import embed


class DataInterface:
    def __init__(self):
        self._sep = os.path.sep
        self.dates = []
        self.datasets_names = []
        self.datasets = {}
        self.shapefile = None
        self.ndvis = {}
        self.nodata_val = -10000.0
        self.nodata_mask = None
#-------------------------------------------------------------------------------

    def add_dataset(self, dataset_path):
        """Adds a dataset.

        Adds a dataset, updates the dates attribute and defines its
        'dataset_name' based on its date and on the type of band.

        Parameters
        ----------
        dataset: str
            full-path to the datasets
        """

        date = dataset_path.split(self._sep)[-1].split("_")[2]
        band = dataset_path.split(self._sep)[-1].split("_")[-1].split(".")[0]
        if date not in self.dates:
            self.dates.append(date)
            self.dates.sort()

        dataset_name = date + "_" + band
        if dataset_name in self.datasets_names:
            warn("Dataset '{}' already added. Ignoring 'add_dataset'"
                .format(dataset_name))
            return

        else:
            self.datasets_names.append(dataset_name)
            self.datasets_names.sort()

            self.datasets[dataset_name] = rio.open(dataset_path)
#-------------------------------------------------------------------------------

    def add_shapefile(self, shapefile_path):
        """Adds a shapefile.

        The shapefile delimits the field which will be analyzed. It can be then
        used in order to clip the dataset.

        Parameters
        ----------
        shapefile_path: str
            full-path to the shapefile
        """

        self.shapefile = gpd.read_file(shapefile_path)
#-------------------------------------------------------------------------------

    def crop_with_shapefile(self):
        """Crops the specified dataset using the previously loaded shapefile
        """

        try:
            shapes = [feature["geometry"] for _, feature in
                self.shapefile.iterrows()]
        except AttributeError:
            raise AttributeError("No shapefile found. Please add a shapefile.")
        else:
            for i, (name, dataset) in enumerate(self.datasets.items()):
                profile = dataset.profile
                data, transform = riom.mask(dataset, shapes, crop=True)

                # Compute nodata_mask (only once!)
                if i == 0:
                    self.nodata_mask = data == self.nodata_val

                profile.update(transform=transform, height=data.shape[1],
                    width=data.shape[2])

                with MemoryFile() as memfile:
                    with memfile.open(**profile) as dataset:
                        dataset.write(data)
                        del data

                    self.datasets[name] = memfile.open()
#-------------------------------------------------------------------------------

    def align_datasets(self, ref_dataset):
        """Aligns all added datasets to a reference dataset.

        Resamples all the added datasets using the specified one as reference in
        order to obtain a perfect pixel-matching.

        Parameters
        ----------
        ref_dataset: str
            name of the dataset that should be used as reference
        """

        out_shape = self.datasets_shape[ref_dataset]
        ref_array = self.datasets[ref_dataset].read(1)
        self.data_mask = (ref_array!=self.no_data_val) & (ref_array!=0)

        for dataset_name in self.datasets_names:
            if not dataset_name.startswith(ref_dataset.split("_")[0]):
                dataset = self.datasets[dataset_name]
                data = dataset.read(out_shape=out_shape,
                    resampling=Resampling.bilinear)

                self.datasets_shape[dataset_name] = data.shape
                self.transforms[dataset_name] = self.transforms[ref_dataset]
                profile = self.datasets[dataset_name].profile
                profile.update(transform=self.transforms[ref_dataset],
                    driver='GTiff', height=data.shape[1], width=data.shape[2])

                with MemoryFile() as memfile:
                    with memfile.open(**profile) as dataset:
                        dataset.write(data)
                        del data

                    self.datasets[dataset_name] = memfile.open()
#-------------------------------------------------------------------------------

    def set_ndvi(self, date):
        """Computes the NDVI-map for the specified date.

        Parameters
        ----------
        date: str
            date at which the NDVI-map should be computed. The date has to be
            given under the format "yyyymmdd"

        Returns
        -------
        numpy.ndarray
            raster containing the NDVI-map for the specified date
        """

        red_name = date + "_" + "red"
        nir_name = date + "_" + "nir"

        try:
            red = self.datasets[red_name].read()
            nir = self.datasets[nir_name].read()
            profile = self.datasets[red_name].profile
        except KeyError as e:
            raise type(e)("Specified dataset has not been added yet.")
        else:
            data = ufunc.ndvi(nir, red)
            data[self.nodata_mask] = self.nodata_val

            with MemoryFile() as memfile:
                with memfile.open(**profile) as dataset:
                    dataset.write(data)
                    del data

                self.ndvis.update({date: memfile.open()})
#-------------------------------------------------------------------------------

    def write_ndvi(self, folder_path, date):
        # Perform some preliminary checks
        if type(date) != str:
            raise Exception("The value specified for parameter 'date' is "
                "invalid. Please specify the date as 'str' (format: "
                "'YYYYMMDD')")
        if not os.path.isdir(folder_path):
            raise Exception("The path where the file should be written does "
                "not exist. Please specify an existing path.")

        else:
            try:
                ndvi = self.ndvis[date]
            except KeyError:
                raise Exception("NDVI for date '{}' does not exist. Please "
                    "specify the date of an existing NDVI.".format(date))
            else:
                profile = self.ndvis[date].profile
                file_path = os.path.join(folder_path, ("ndvi_" + date + ".tif"))
                with rio.open(file_path, mode='w', **profile) as dst:
                    dst.write(ndvi.read())

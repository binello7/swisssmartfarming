from rasterio import MemoryFile, Affine
from rasterio.warp import Resampling
from warnings import warn
import geopandas as gpd
import numpy as np
import rasterio as rio
import rasterio.mask as riom
import utils.functions as ufunc

class DataInterface:
    def __init__(self):
        self.dates = []
        self.datasets_names = []
        self.datasets = {}
        self.shapefile_name = None
        self.shapefile = None
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

        date = dataset_path.split("/")[-1].split("_")[2]
        band = dataset_path.split("/")[-1].split("_")[-1].split(".")[0]
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

            with rio.open(dataset_path) as source:
                self.datasets[dataset_name] = source
#-------------------------------------------------------------------------------

    def add_shapefile(self, shapefile):
        """Adds a shapefile.

        The shapefile delimits the field which will be analyzed. It can be then
        used in order to clip the dataset.

        Parameters
        ----------
        shapefile: str
            full-path to the shapefile
        """

        self.shapefile = gpd.read_file(shapefile)
        self.shapefile_name = shapefile.split("/")[-1].split(".")[0]
#-------------------------------------------------------------------------------

    def crop_dataset(self, dataset_name):
        """Crops the specified dataset using the previously loaded shapefile

        Parameters
        ----------
        dataset_name: str
            name of the dataset to crop
        """

        try:
            dataset = self.datasets[dataset_name]
        except KeyError as e:
            raise type(e)(str(e) + ' not in datasets')
        else:
            try:
                shapes = [feature["geometry"]
                    for _, feature in self.shapefile.iterrows()]
            except AttributeError:
                raise Exception("Shapefile has not been added yet.")
            else:
                data, transform = riom.mask(dataset, shapes, crop=True)
                self.transforms[dataset_name] = transform
                self.datasets_shape[dataset_name] = data.shape

                profile = self.datasets[dataset_name].profile
                profile.update(transform=transform, driver='GTiff',
                    height=data.shape[1], width=data.shape[2])

                with MemoryFile() as memfile:
                    with memfile.open(**profile) as dataset:
                        dataset.write(data)
                        del data

                    self.datasets[dataset_name] = memfile.open()
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

    def ndvi(self, dataset_name):
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
        r_band_nb = 3   # NOTE: wavelenght ordering instead of MicaSense ordering
        nir_band_nb = 5 # NOTE: wavelenght ordering instead of MicaSense ordering
        try:
            red = self.datasets[dataset_name].read(r_band_nb)
            nir = self.datasets[dataset_name].read(nir_band_nb)
        except KeyError as e:
            raise type(e)("Specified dataset has not been added yet.")
        else:
            return ufunc.ndvi(nir, red)

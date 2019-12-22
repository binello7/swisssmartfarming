import rasterio as rio
import rasterio.mask as riom
import rasterio.plot as riop
from rasterio.transform import Affine
import matplotlib.pyplot as plt
import fiona as fio
import numpy as np
import geopandas as gpd
from shapely.geometry import Polygon
import os

from IPython import embed

class DatasetManipulator:
    def __init__(self, dataset_path):
        self.dataset_path = dataset_path
        self.dataset_path_padded = None
        self.dataset_name = "_".join(dataset_path.split("/")[-1].split("_")[:3])
        self.dataset = rio.open(dataset_path)
        self.transform = self.dataset.transform
        self.crs = self.dataset.crs
        self.Xres = self.transform[0]
        self.Yres = -self.transform[4]
        self.gridspacing_x = None
        self.gridspacing_y = None
        self.grid = None
        self.grid_bounds = None
        self.mask = None
        self.mask_path = None


    def create_grid(self, outer_shapefile, gridspacing_x=256, gridspacing_y=256):
        """Creates a grid and sets it to the grid dataset attribute.

        Given the shapefile confining the dataset geographic area , creates theembed()

        geometry of a grid covering it. If the grid does not fit exactly in the
        shapefile, the grid will have an extra cell in order to cover all of the
        area. The x- and y cell spacings of the grid are determined by the
        parameters <gridspacing_x> and <gridspacing_y> which have to be given
        in pixels.

        Parameters
        ----------
        outer_shapefile: geopandas.geodataframe.GeoDataFrame
            shapefile confining the area
        gridspacing_x: int
            cell width in pixels
        gridspacing_y: int
            cell height in pixels
        """

        # get the xmin, xmax, ymin, ymax of the shapefile bounding-box
        xmin, ymin, xmax, ymax = outer_shapefile.geometry.total_bounds

        # set the x and y-spacing attributes
        self.gridspacing_x = gridspacing_x
        self.gridspacing_y = gridspacing_y

        # convert the cell-dimensions from px to units
        gridspacing_x = gridspacing_x * self.Xres
        gridspacing_y = gridspacing_y * self.Yres

        # get x and y number of cells
        nx = (xmax - xmin) // gridspacing_x # //: integer division
        if (xmax - xmin) % gridspacing_x != 0:
            nx = nx + 1

        ny = (ymax - ymin) // gridspacing_y
        if (ymax - ymin) % gridspacing_y != 0:
            ny = ny + 1

        # get the new xmax, ymin
        xmax = xmin + nx * gridspacing_x
        ymin = ymax - ny * gridspacing_y

        # set the grid bounds
        self.grid_bounds = (xmin, ymin, xmax, ymax)


        # get the x and y coordinates of the grid
        x_coord = list(np.arange(xmin, xmax, gridspacing_x))
        y_coord = list(np.arange(ymin, ymax, gridspacing_y))
        y_coord.reverse()

        # generate the polygon object determined by the 4 corners of each cell
        polygons = []
        for y in y_coord[:-1]:
            for x in x_coord[:-1]:
                polygons.append(Polygon([(x, y),
                                         (x+gridspacing_x, y),
                                         (x+gridspacing_x, y-gridspacing_y),
                                         (x, y-gridspacing_y)]))

        self.grid = gpd.GeoDataFrame({'geometry': polygons,
                                      'grid_idx': range(0, len(polygons))})


    def pad_geotiff_from_grid(self):
        """Uses the previously created grid to pad the source raster dataset.

        The previously created grid is used to slice the raster dataset into N
        smaller rasters all with the same shape. This functions pads the source
        raster with 'zeros' in order to obtain N images all with the same shape.
        """

        if self.grid is None:
            raise NotImplementedError('Grid not created yet')

        # get the total height and width in pixels of the dataset after padding
        tot_px_x = int(round((self.grid_bounds[2] - self.grid_bounds[0])
            / self.Xres))
        tot_px_y = int(round((self.grid_bounds[3] - self.grid_bounds[1])
            / self.Yres))

        # load the data for every band and shape them as (height, width, band)
        array = self.dataset.read()

        # for rgb do not consider alpha channel
        if 'rgb' in self.dataset_name:
            array = array[:-1,:,:]

        pad_hor = tot_px_x - array.shape[2]
        pad_ver = tot_px_y - array.shape[1]

        array = np.pad(array, ((0, 0), (0, pad_ver), (0, pad_hor)),
            mode='constant', constant_values=0)

        self.dataset_path_padded = (os.path.join('/'
            .join(self.dataset_path.split('/')[:-1]),
            self.dataset_path.split('/')[-1].split('.')[0])
            + '_padded.tif')

        self.dataset = rio.open(
            self.dataset_path_padded,
            'w',
            driver='Gtiff',
            height=array.shape[1],
            width=array.shape[2],
            count=array.shape[0],
            dtype=array.dtype,
            crs=self.crs,
            transform=self.transform
        )

        self.dataset.write(array, tuple(np.arange(1, array.shape[0] + 1)))
        self.dataset.close()
        self.dataset = rio.open(self.dataset_path_padded)


    def get_pair_from_idx(self, grid_idx):
        """Returns the pair (image, mask) at the specified grid location.

        Crops the GeoTIFF using the generated grid and returns the image and its
        corresponding mask defined by the specified grid index.

        Parameters
        ----------
        grid_idx: int
            integer corresponding to the index of the desired raster region

        Returns
        -------
        img: numpy.ndarray
            numpy-array representing the requested image with shape (H, W, C)
        maks: numpy.ndarray
            numpy-array representing the plant mask at the specified location
            with shape (H, W)
        """

        if self.grid is None:
            raise NotImplementedError("The grid hasn't been created yet")

        if self.dataset_path_padded is None:
            raise NotImplementedError("The raster hasn't been padded. Perform"
                " padding in order to insure coherent images dimensions.")

        if self.mask is None:
            raise NotImplementedError("The raster hasn't been masked. Impossible"
                " to retunr the pair (img, mask)")

        # get the coordinates of top-left and bottom-right corners into a tuple
        boundary = self.grid["geometry"][grid_idx]
        boundary = boundary.exterior.xy
        top_left = (boundary[0][0], boundary[1][0])
        bottom_right = (boundary[0][1], boundary[1][2])

        # get the indexes of the pixels at top-left / bottom-right coordinates
        row_start, col_start = rio.transform.rowcol(self.transform, top_left[0],
            top_left[1])
        row_end, col_end = rio.transform.rowcol(self.transform, bottom_right[0],
            bottom_right[1])

        # convert the indexes to integers
        row_start = int(row_start); row_end = int(row_end)
        col_start = int(col_start); col_end = int(col_end)

        # get the values of the pixel within the boundary
        img = self.dataset.read()[:, row_start:row_end, col_start:col_end]
        img = np.moveaxis(img, 0, 2)

        mask = self.mask.read()[:,row_start:row_end, col_start:col_end]
        mask = np.moveaxis(mask, 0, 2)
        mask = np.squeeze(mask, axis=2)

        # convert the image to channels last and return it
        return img, mask


    def create_mask_from_shapes(self, shapefile):
        # TODO: write documentation for the method
        # if self.dataset_path_padded is None:
        #     raise NotImplementedError('Dataset has to be padded with the grid '
        #         'before performing the masking operation')

        with fio.open(shapefile, "r") as shp:
            shapes = []
            for feature in shp:
                if feature["geometry"] != None:
                    shapes.append(feature["geometry"])

        mask, _ = riom.mask(self.dataset, shapes)
        mask = mask[0,:,:]
        mask[mask!=0] = 1

        self.mask_path = (os.path.join('/'
            .join(self.dataset_path.split('/')[:-1]),
            self.dataset_path.split('/')[-1].split('.')[0])
            + '_mask.tif')

        self.mask = rio.open(
            self.mask_path,
            'w',
            driver='Gtiff',
            height=mask.shape[0],
            width=mask.shape[1],
            count=1,
            dtype=mask.dtype,
            crs=self.crs,
            transform=self.transform
        )

        self.mask.write(mask, 1)
        self.mask.close()
        self.mask = rio.open(self.mask_path)
#-------------------------------------------------------------------------------

    def visualize_dataset(self, with_grid=True):
        if with_grid:
            if self.grid is None:
                raise ValueError("Grid value is '{}'. Grid not created yet."
                    .format(self.grid))

        cells = self.grid["geometry"]

        fig, axs = plt.subplots()

        if self.dataset is not None:
            riop.show(self.dataset, ax=axs)

        if self.mask is not None:
            riop.show(self.mask, ax=axs, alpha=0.5)

        for i, cell in enumerate(cells):
            x, y = cell.exterior.xy
            plt.plot(x, y)
            xm = (x[1] - x[0]) / 2
            ym = (y[2] - y[1]) / 2
            text = str(self.grid['grid_idx'][i])
            plt.text(x[0] + xm, y[0] + ym, text, color='r')

        plt.show()

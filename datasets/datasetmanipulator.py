import rasterio as rio
from rasterio.transform import Affine
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


    # def load_geotiff():

    def set_grid(self, outer_shapefile, gridspacing_x=256, gridspacing_y=256):
        """Creates a grid and sets it to the grid dataset attribute.

        Given the shapefile confining the dataset geographic area , creates the
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


    def get_img_from_boundary(self, boundary):
        """Crops a GeoTIFF using a rectangular boundary.

        Given Polygon object representing a rectangular boundary, crops a bigger
        GeoTIFF along the boundary and returns the sliced image array.

        Parameters
        ----------
        boundary: shapely.geometry.polygon.Polygon
            shapely-Polygon representing cropping boundary

        Returns
        -------
        numpy.ndarray
            numpy-array of the image with shape (H, W, C)
        """

        # get the coordinates of top-left and bottom-right corners into a tuple
        boundary = boundary.exterior.xy
        top_left = (boundary[0][0], boundary[1][2])
        bottom_right = (boundary[0][1], boundary[1][0])

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

        # convert the image to channels last and return it
        return np.moveaxis(img, 0, 2)

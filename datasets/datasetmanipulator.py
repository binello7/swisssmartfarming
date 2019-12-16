import rasterio as rio
import numpy as np
import geopandas as gpd
from shapely.geometry import Polygon

from IPython import embed

class DatasetManipulator:
    def __init__(self, geotiff_path, dataset_name):
        self.dataset = rio.open(geotiff_path)
        self.dataset_name = dataset_name
        self.transform = self.dataset.transform
        self.Xres = self.dataset.transform[0]
        self.Yres = -self.dataset.transform[4]
        self.x_spacing = None
        self.y_spacing = None
        self.grid = None

    def set_grid(self, outer_shapefile, x_spacing=256, y_spacing=256):
        """Creates a grid and sets it to the grid dataset attribute.

        Given the shapefile confining the dataset geographic area , creates the geometry of a
        grid covering it. If the grid does not fit exactly in the shapefile, the grid
        will have an extra cell in order to cover all of the area. The x- and y
        cell spacings of the grid are determined by the parameters <x_spacing> and
        <y_spacing> which have to be given in pixels.

        Parameters
        ----------
        outer_shapefile: geopandas.geodataframe.GeoDataFrame
            shapefile confining the area
        x_spacing: int
            cell width in pixels
        y_spacing: int
            cell height in pixels
        """

        # get the xmin, xmax, ymin, ymax of the shapefile bounding-box
        xmin, ymin, xmax, ymax = outer_shapefile.geometry.total_bounds

        # set the x and y-spacing attributes
        self.x_spacing = x_spacing
        self.y_spacing = y_spacing

        # convert the cell-dimensions from px to units
        x_spacing = x_spacing * self.Xres
        y_spacing = y_spacing * self.Yres

        # get x and y number of cells
        nx = (xmax - xmin) // x_spacing # //: integer division
        if (xmax - xmin) % x_spacing != 0:
            nx = nx + 1

        ny = (ymax - ymin) // y_spacing
        if (ymax - ymin) % y_spacing != 0:
            ny = ny + 1

        # get the new xmax, ymax
        xmax = xmin + nx * x_spacing
        ymax = ymin + ny * y_spacing

        # get the x and y coordinates of the grid
        x_coord = list(np.arange(xmin, xmax, x_spacing))
        y_coord = list(np.arange(ymin, ymax, y_spacing))

        # generate the polygon object determined by the 4 corners of each cell
        polygons = []
        for y in y_coord[:-1]:
            for x in x_coord[:-1]:
                polygons.append(Polygon([(x, y),
                                         (x+x_spacing, y),
                                         (x+x_spacing, y+y_spacing),
                                         (x, y+y_spacing)]))

        self.grid = gpd.GeoDataFrame({'geometry': polygons,
                                      'grid_idx': range(0, len(polygons))})


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
        img = self.dataset.read()[:-1, row_start:row_end, col_start:col_end]

        # convert the image to channels last and return it
        return np.moveaxis(img, 0, 2)

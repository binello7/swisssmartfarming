import rasterio as rio
import numpy as np
import geopandas as gpd
from shapely.geometry import Polygon

from IPython import embed

class DatasetManipulator:
    def __init__(self, geotiff_path, dataset_name):
        self.dataset = rio.open(geotiff_path)
        self.dataset_name = dataset_name
        self.Xres = self.dataset.transform[0]
        self.Yres = -self.dataset.transform[4]
        self.grid = None

    def create_grid(self, outer_shapefile, x_spacing=256, y_spacing=256):
        """creates a grid over an area determined by a shapefile.

        Given a geographic area confined by a shapefile, creates the geometry of a
        grid over it. If the grid does not fit exactly in the shapefile, the grid
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
        y_coord.reverse()

        # generate the polygone object determined by the 4 corners of each cell
        polygons = []
        for y in y_coord:
            for x in x_coord:
                polygons.append(Polygon([(x, y),
                                         (x+x_spacing, y),
                                         (x+x_spacing, y-y_spacing),
                                         (x, y-y_spacing)]))

        self.grid = gpd.GeoDataFrame({'geometry': polygons,
                                      'grid_idx': range(0, len(polygons))})




        # import the polygone of a field which will be sliced into a grid. If
        # the grid doesn't fall exactly on the outer corner of the polygone we
        # add a cell to the grid

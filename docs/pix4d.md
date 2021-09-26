# Pix4D

## Processing

### 1\. Initial Processing

#### Calibration

- Calibration Method

  - Alternative: Optimized for aerial nadir images with accurate geolocation, low texture content and relatively flat terrain, fields for example. The calibration will fail if the number of oblique images (>35 deg) in the dataset is higher than 5%. Furthermore, at least 75% of all images have to be geolocated

  - Camera Optimization

    - Internal Parameters Optimization

      - All (default): Optimizes all the internal camera parameters
      - **All Prior**: Forces the optimal internal parameters to be close to the initial values

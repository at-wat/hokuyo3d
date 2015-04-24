^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package hokuyo3d
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Forthcoming
-----------

0.1.0 (2015-04-24)
------------------
* direct PointCloud2 message encoding
* adds list of contributors
* adds feature to publish PointCloud2 message
  "~/hokuyo_cloud" and "~/hokuyo_cloud2" are published on demand.
  Added pursuant to yukkysaito's request.
* adds parameter to set data output cycle
  ~output_cycle (string, default: field)
  Sets output timing to end of frame, field or line.
  Added pursuant to yukkysaito's request.
* Add invalid range parameter
  This commit is modified for merging by at-wat.
  Parameter name invalid_range was changed to range_min.
* fixes a bug in which output data doesn't have all points
  Fixed pursuant to yukkysaito's report.
* code refactoring
* skips invalid data without 'VSSP' mark
* stops data stream correctly before exit
* scales aux data
* estimates real measurement time from timestamp
* receives aux data and publishes Imu and MagneticField message
* add README.md
* Initial commit
* Contributors: Atsushi Watanabe, yukihiro saito

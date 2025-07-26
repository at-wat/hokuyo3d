^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package hokuyo3d
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Forthcoming
-----------
* Migrate to Alpine ROS 1 (`#87 <https://github.com/at-wat/hokuyo3d/issues/87>`_)
* Update assets to v0.6.4 (`#86 <https://github.com/at-wat/hokuyo3d/issues/86>`_)
* Update assets to v0.6.3 (`#85 <https://github.com/at-wat/hokuyo3d/issues/85>`_)
* Update assets to v0.6.2 (`#84 <https://github.com/at-wat/hokuyo3d/issues/84>`_)
* Update assets to v0.6.1 (`#83 <https://github.com/at-wat/hokuyo3d/issues/83>`_)
* Update assets to v0.6.0 (`#82 <https://github.com/at-wat/hokuyo3d/issues/82>`_)
* Update assets to v0.5.2 (`#81 <https://github.com/at-wat/hokuyo3d/issues/81>`_)
* Update assets to v0.5.1 (`#80 <https://github.com/at-wat/hokuyo3d/issues/80>`_)
* Update assets to v0.5.0 (`#79 <https://github.com/at-wat/hokuyo3d/issues/79>`_)
* Update assets to v0.4.2 (`#78 <https://github.com/at-wat/hokuyo3d/issues/78>`_)
* Update assets to v0.4.1 (`#77 <https://github.com/at-wat/hokuyo3d/issues/77>`_)
* Update assets to v0.4.0 (`#76 <https://github.com/at-wat/hokuyo3d/issues/76>`_)
* Update assets to v0.3.4 (`#75 <https://github.com/at-wat/hokuyo3d/issues/75>`_)
* Update assets to v0.3.3 (`#73 <https://github.com/at-wat/hokuyo3d/issues/73>`_)
* Update assets to v0.3.2 (`#72 <https://github.com/at-wat/hokuyo3d/issues/72>`_)
* Update assets to v0.3.1 (`#71 <https://github.com/at-wat/hokuyo3d/issues/71>`_)
* Update assets to v0.3.0 (`#70 <https://github.com/at-wat/hokuyo3d/issues/70>`_)
* Update assets to v0.2.0 (`#69 <https://github.com/at-wat/hokuyo3d/issues/69>`_)
* Update assets to v0.1.5 (`#66 <https://github.com/at-wat/hokuyo3d/issues/66>`_)
* Update assets to v0.1.4 (`#65 <https://github.com/at-wat/hokuyo3d/issues/65>`_)
* Update dockerignore (`#64 <https://github.com/at-wat/hokuyo3d/issues/64>`_)
* Update assets to v0.1.3 (`#63 <https://github.com/at-wat/hokuyo3d/issues/63>`_)
* Update assets to v0.1.2 (`#62 <https://github.com/at-wat/hokuyo3d/issues/62>`_)
* Migrate to GitHub Actions (`#60 <https://github.com/at-wat/hokuyo3d/issues/60>`_)
* Update assets to v0.0.10 (`#59 <https://github.com/at-wat/hokuyo3d/issues/59>`_)
* Update assets to v0.0.9 (`#57 <https://github.com/at-wat/hokuyo3d/issues/57>`_)
* Update CI scripts (`#58 <https://github.com/at-wat/hokuyo3d/issues/58>`_)
* Bump cmake minimum version to 3.0.2 (`#55 <https://github.com/at-wat/hokuyo3d/issues/55>`_)
* Contributors: Atsushi Watanabe

0.2.1 (2020-04-07)
------------------
* Fix bot comment on prerelease test (`#52 <https://github.com/at-wat/hokuyo3d/issues/52>`_)
* Update CI scripts (`#49 <https://github.com/at-wat/hokuyo3d/issues/49>`_)
* Support Boost 1.69 (`#48 <https://github.com/at-wat/hokuyo3d/issues/48>`_)
* Update pointer align style (`#47 <https://github.com/at-wat/hokuyo3d/issues/47>`_)
* Update assets to v0.0.8 (`#46 <https://github.com/at-wat/hokuyo3d/issues/46>`_)
* Update assets to v0.0.7 (`#45 <https://github.com/at-wat/hokuyo3d/issues/45>`_)
* Update assets to v0.0.6 (`#44 <https://github.com/at-wat/hokuyo3d/issues/44>`_)
* Ignore gh-pr-comment failure (`#42 <https://github.com/at-wat/hokuyo3d/issues/42>`_)
* Fix CI bot (`#41 <https://github.com/at-wat/hokuyo3d/issues/41>`_)
* Fix travis build branch (`#40 <https://github.com/at-wat/hokuyo3d/issues/40>`_)
* Contributors: Atsushi Watanabe

0.2.0 (2018-03-24)
------------------
* Fix timestamp estimation (`#35 <https://github.com/at-wat/hokuyo3d/issues/35>`_)
  * Fix timestamp estimation
  * Decrease timstamp base lpf
  * Drop timestamp jump back
  * Add median filter to the timestamp estimation
  * Add debug output of estimated timestamp epoch
  * Add parameter to allow timstamp jump back (default: false)
* Install launch files (`#32 <https://github.com/at-wat/hokuyo3d/issues/32>`_)
* Fix horizontal table precision bug (`#30 <https://github.com/at-wat/hokuyo3d/issues/30>`_)
* Fix connection start timing (`#29 <https://github.com/at-wat/hokuyo3d/issues/29>`_)
  * Suppress `Transmission timeout` error from the sensor
* Support auto reset setting (`#27 <https://github.com/at-wat/hokuyo3d/issues/27>`_)
  * Support auto reset setting
  * Don't send auto reset command if the parameter is not given
* Support vertical interlace added on VSSP2.1 (`#25 <https://github.com/at-wat/hokuyo3d/issues/25>`_)
* Fix local variable naming style (`#26 <https://github.com/at-wat/hokuyo3d/issues/26>`_)
* Use timer instead of polling (`#23 <https://github.com/at-wat/hokuyo3d/issues/23>`_)
* Remove needless semicolons (`#22 <https://github.com/at-wat/hokuyo3d/issues/22>`_)
* Fix CI bot setting (`#24 <https://github.com/at-wat/hokuyo3d/issues/24>`_)
* Fix aux data factors. (`#18 <https://github.com/at-wat/hokuyo3d/issues/18>`_)
* Add a sample launch file to publish frames. (`#17 <https://github.com/at-wat/hokuyo3d/issues/17>`_)
* Fix socket read buffer size. (`#14 <https://github.com/at-wat/hokuyo3d/issues/14>`_)
* Add const attribute to constant things. (`#16 <https://github.com/at-wat/hokuyo3d/issues/16>`_)
* Fix naming style of local variables. (`#15 <https://github.com/at-wat/hokuyo3d/issues/15>`_)
* Fix coding styles. (`#12 <https://github.com/at-wat/hokuyo3d/issues/12>`_)
  - fix coding rules
  - fix naming styles
  - add roslint test
* Fix mag frame_id. (`#13 <https://github.com/at-wat/hokuyo3d/issues/13>`_)
* Change IMU coordinate frame. (`#10 <https://github.com/at-wat/hokuyo3d/issues/10>`_)
* Add build test on Travis. (`#11 <https://github.com/at-wat/hokuyo3d/issues/11>`_)
* Contributors: Atsushi Watanabe

0.1.1 (2016-09-15)
------------------
* updates e-mail address of the author
* adds error-message packet handling
* Merge branch 'lgerardSRI-master'
* Install the hokuyo3d executable
* Contributors: Atsushi Watanabe, Leonard Gerard

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

^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package sicks300
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.0.5 (2014-11-05)
------------------
* Merge pull request `#5 <https://github.com/strands-project/sicks300/issues/5>`_ from strands-project/upstream_merge
  Upstream merge
* Merge branch 'master' of https://github.com/bohlender/sicks300 into upstream_merge
* Merge pull request `#3 <https://github.com/strands-project/sicks300/issues/3>`_ from cburbridge/master
  Allow user to select reduced field of view.
* Merge pull request `#4 <https://github.com/strands-project/sicks300/issues/4>`_ from strands-project/prepare_for_release
  Prepare for release
* Merge pull request `#2 <https://github.com/strands-project/sicks300/issues/2>`_ from strands-project/merge_prepare_for_release
  Merge prepare for release
* Merge remote-tracking branch 'origin/prepare_for_release' into merge_prepare_for_release
* merged
* added Marc Hanheide as additional maintainer
* - added changelog
  - added <cstddef> for new gcc
  - added unistd.h for gcc
* Allow user to select reduced field of view.
  This commit replaces the bool 'reduced_fov' parameter with double 'field_of_view'. This allows the user to select what angle of view the laser should publish. Setting this to 180 will have the same effect as setting reduced_fov to 1 previously. Setting it to 260 will remove 5 degrees from the start and end of the scan.
* Contributors: Dimitri Bohlender, Marc Hanheide, cburbridge

* - added changelog
  - added <cstddef> for new gcc
  - added unistd.h for gcc
* Contributors: Marc Hanheide

* Merge pull request `#2 <https://github.com/strands-project/sicks300/issues/2>`_ from larics/master
  Catkinized the package
* Updated CMakeLists.txt and package.xml.
* Catkinized the package.
* Update README.md
* Fixed typo and added more explicit credits.
* Modified description and authors in manifest file
* Update README.md
* Create README.md
* - Adapted original implementation to support both the old (v.1.02) and the new (v.1.03) protocols for continuous data output of the SICK S300 Professional
  - Fixed a bug which caused the header start to be off (this caused unnecessary CRC failures)
  - Adapted copyrights/license stuff
* 
* 
* 
* 
* Contributors: Damjan Miklic, Dimitri Bohlender, dbohlender, torstenfiolka

* Merge pull request `#2 <https://github.com/strands-project/sicks300/issues/2>`_ from larics/master
  Catkinized the package
* Updated CMakeLists.txt and package.xml.
* Catkinized the package.
* Update README.md
* Fixed typo and added more explicit credits.
* Modified description and authors in manifest file
* Update README.md
* Create README.md
* - Adapted original implementation to support both the old (v.1.02) and the new (v.1.03) protocols for continuous data output of the SICK S300 Professional
  - Fixed a bug which caused the header start to be off (this caused unnecessary CRC failures)
  - Adapted copyrights/license stuff
* 
* 
* 
* 
* Contributors: Damjan Miklic, Dimitri Bohlender, dbohlender, torstenfiolka

^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package roslint
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.10.0 (2015-08-07)
-------------------
* Updated pep8 from 1.4.7a0 to 1.6.2
* Updated cpplint to the latest version (https://github.com/google/styleguide/tree/554223dc5432f9bd67951cde662f7a023c512dec)
* Fix for falsely reporting do-whiles as an error
* Fixes for access control in structs
* Contributors: Alex Henning, Mike Purvis

0.9.3 (2015-01-06)
------------------
* Don't hang on header outside "include" dir.
* Contributors: Mike Purvis

0.9.2 (2014-03-31)
------------------
* Better implementation of roslint_add_test
* Simple implementation of XML results output
* roslint roslints itself
* Contributors: Mike Purvis

0.9.1 (2014-02-18)
------------------
* Add roslint_add_test function
* Run the include-line checks with errors suppressed. This kills spurious build/include_what_you_use errors.
* Contributors: Mike Purvis

0.9.0 (2014-02-17)
------------------
* Allow a trailing semicolon after closing brace.
* Add more tolerance for braces as array initializers, and eliminate the warning about access control labels.
* Rename python library to roslint, to play better.
* Use templated extras file to find roslint scripts without rosrun. 
* Max length override for pep8; remove roslint custom shout.
* Add some overrides in an effort to comply better with ROS C++ Style.
* Contributors: Mike Purvis

0.0.1 (2013-10-17)
------------------
* Basic initial release, with roslint_python, roslint_cpp, and roslint_custom macros included.
* pep8 and cpplint linters packaged-in.

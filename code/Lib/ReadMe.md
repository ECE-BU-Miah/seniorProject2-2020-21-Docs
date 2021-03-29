# Library Files (*lib*) ReadMe
## Overview
This Read me is for library directory where any aditional library files other than those native to c, gcc, and/or linux are stored.  This direcory can contain ether imported librarys or custom ones created for this project wich follow the naming convention layed out bellow.

## Custom Library Naming Conventions
* ### File Name
    * (Camel Case) - ex. myFile
* ### Function Name
    * {File Name}\_(PascalCase) - ex. myFile\_MyFunction
* ### Global Variable Names
    * {File Name}\_(Camel Case) - ex. myFile\_myGlobal
* ### Definition Names
    * {File Name (Upper Snake Case)}\_(Upper Snake Case) - ex. MY_FILE_MY_DEFINE

* ### Exceptions:
    * _core.h_ is allowed to violate naming rules do to explicit relation to this project and ubiquitus use of the contents of it
    * _extraMath.h_ is allowed to deviate from the rules where needed to act more simularley to the base C math library it is extending
# pioneer
Intro to component here

## Install
Install Aria lib for https://github.com/cinvesrob/Aria.git with changes in:
- In src/ArJoyHandler_LIN line 44     "if(myOldJoyDesc > 0)" to  if(fileno(myOldJoyDesc) > 0) 
- In src/ArLog.cpp delete all "if (err < sys_nerr - 1)" and change all "errorString = sys_errlist[err]" to "errorString = strerror(err)"

## Configuration parameters
As any other component, *pioneer* needs a configuration file to start. In
```
etc/config
```
you can find an example of a configuration file. We can find there the following lines:
```
Port Robot

port=/dev/ttyUSB0
```

## Starting the component
To avoid changing the *config* file in the repository, we can copy it to the component's home directory, so changes will remain untouched by future git pulls:

```
cd <pioneer's path> 
```
```
cp etc/config config
```

After editing the new config file we can run the component:

```
bin/pioneer config
```

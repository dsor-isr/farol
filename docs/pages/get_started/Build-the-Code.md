## Compile the code

### First Time
```
cd ~/<path_to_workspace>/<catkin_ws>/
catkin build
```
## After first time and using alias
```
S
farol_cb
```

Note: This can be done in any folder because the alias is the following:

```
alias farol_cb='roscd; catkin build; cd $OLDPWD'
```

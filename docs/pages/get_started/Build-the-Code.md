## Compile the code

### First Time
```
cd ~/<path_to_workspace>/<catkin_ws>/
catkin build
```

Note: replace `/<path_to_workspace>` with the folder where you put you catkin_ws inside (for example `/dsor`). If you put in your home folder, then this variable should be left empty!

### After first time and using alias
```
S
farol_cb
```

Note: This can be done in any folder because the alias is the following:

```
alias farol_cb='roscd; catkin build; cd $OLDPWD'
```


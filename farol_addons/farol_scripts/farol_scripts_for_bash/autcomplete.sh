function _roscomplete_packages {
    local arg opts
    arg="${COMP_WORDS[COMP_CWORD]}"

    if [[ $COMP_CWORD == 1 ]]; then
        _roscomplete_pkg "${arg}"
    elif [[ $COMP_CWORD == 0 ]]; then
        opts=`rospack list-names`
        COMPREPLY=($(compgen -W "$opts" -- ${arg}))
    fi
}

complete -F "_roscomplete_packages" "clean_pkg_from_ws"
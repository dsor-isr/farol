# Pull the latest update in a repo and the corresponding submodules
alias mgit_pull='git pull && git submodule update --init --recursive'

# Get the status of the repository and the version of the submodules
alias mgit_status='git status && git submodule status --recursive'

# ------------------------------------------ #
# Standard commands, but put as farol alias #
# ------------------------------------------ #

# Push the local tags to the remote repository
alias mgit_push_tag='git push origin --tags'

# Get a general description of the repository
alias mgit_describe='git describe'

# Get the git log with commits
alias mgit_log='git log'
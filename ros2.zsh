# --- ROS Environment Management ---
# Provides functions to switch between ROS environments, ensuring a clean switch.

_ros_cleanup() {
    # 1. Unset all known ROS-related environment variables to avoid conflicts.
    # This is important for variables that don't have a path-like structure.
    unset ROS_DISTRO ROS_PACKAGE_PATH AMENT_PREFIX_PATH COLCON_PREFIX_PATH
    unset ROS_PYTHON_VERSION ROS_LOCALHOST_ONLY ROS_VERSION ROS_MASTER_URI ROS_IP

    # 2. Clean key environment paths of all ROS-related entries.
    # The function removes any path segment that starts with '/opt/ros/'.
    # This is more robust than checking for specific versions.
    _clean_ros_from_path() {
        local original_path="$1"
        if [ -z "$original_path" ]; then return; fi
        local cleaned_path=""
        # Loop through path elements separated by colons.
        for p in ${(s/:/)original_path}; do
            # If the path segment does NOT start with '/opt/ros/', keep it.
            if [[ "$p" != /opt/ros/* ]]; then
                if [[ -z "$cleaned_path" ]]; then
                    cleaned_path="$p"
                else
                    cleaned_path="$cleaned_path:$p"
                fi
            fi
        done
        echo "$cleaned_path"
    }

    # Clean all relevant paths by re-exporting them after cleaning.
    export PATH=$(_clean_ros_from_path "$PATH")
    export PYTHONPATH=$(_clean_ros_from_path "$PYTHONPATH")
    export CMAKE_PREFIX_PATH=$(_clean_ros_from_path "$CMAKE_PREFIX_PATH")
    export LD_LIBRARY_PATH=$(_clean_ros_from_path "$LD_LIBRARY_PATH")
    export PKG_CONFIG_PATH=$(_clean_ros_from_path "$PKG_CONFIG_PATH")

    # 3. Undefine potentially conflicting shell functions from ROS 2.
    # This prevents errors where a ROS 2 function is called in a ROS 1 environment.
    if typeset -f ros2 > /dev/null; then unfunction ros2; fi
    if typeset -f colcon > /dev/null; then unfunction colcon; fi
    if typeset -f ament > /dev/null; then unfunction ament; fi
    if typeset -f _ros2_local_setup > /dev/null; then unfunction _ros2_local_setup; fi
}

# Default to ROS1 (Noetic) on shell startup.
source /opt/ros/noetic/setup.zsh

# Function to switch to ROS1 (Noetic)
one() {
    echo "Switching to ROS1 (Noetic)..."
    _ros_cleanup
    source /opt/ros/noetic/setup.zsh
}

# Function to switch to ROS2 (Foxy)
two() {
    echo "Switching to ROS2 (Foxy)..."
    _ros_cleanup
    source /opt/ros/foxy/setup.zsh
}

# --- Custom Prompt for ROS ---
# This section dynamically updates the Zsh prompt to show the current ROS environment.
# It checks the ROS_DISTRO variable before each prompt and displays (ros1) or (ros2).

# Function to set the prompt prefix based on ROS_DISTRO
update_ros_prompt() {
  case "$ROS_DISTRO" in
    "noetic")
      # Set prefix for ROS1 (Noetic)
      ROS_PROMPT_PREFIX="%{$fg[green]%}(ros1)%{$reset_color%} "
      ;;
    "foxy")
      # Set prefix for ROS2 (Foxy)
      ROS_PROMPT_PREFIX="%{$fg[yellow]%}(ros2)%{$reset_color%} "
      ;;
    *)
      # No prefix if no known ROS is active
      ROS_PROMPT_PREFIX=""
      ;;
  esac
}

# Register the function to run before each prompt is displayed
autoload -U add-zsh-hook
add-zsh-hook precmd update_ros_prompt

# Define the final prompt structure, including the dynamic ROS prefix.
# This re-creates the default 'robbyrussell' theme prompt with our addition.
PROMPT='$ROS_PROMPT_PREFIX%{$fg_bold[cyan]%}%c%{$reset_color%} $(git_prompt_info)'

# Simulate ROS1's roscd functionality for ROS2
ros2cd() {
  if [ -z "$1" ]; then
    echo "Usage: ros2cd <package_name>"
    compdef _ros2cd_completion ros2cd
    return 1
  fi
  local pkg_path=$(ros2 pkg prefix "$1" 2>/dev/null)
  if [ -z "$pkg_path" ]; then
    echo "Package '$1' not found"
    return 1
  fi
  cd "$pkg_path"
}

# Completion function for ros2cd
_ros2cd_completion() {
  local -a packages
  packages=($(ros2 pkg list 2>/dev/null))
  compadd -a packages
}

# Register the completion function for ros2cd
compdef _ros2cd_completion ros2cd
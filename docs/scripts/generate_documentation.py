import os, sys
import shutil
import subprocess
import ruamel.yaml
import argparse

# Get the arguments from the execution (and check if we want to deploy to github pages)
parser = argparse.ArgumentParser(description='Documentation Generation')
deploy = parser.add_subparsers(dest='command').add_parser('deploy', help='Deploy the documentation to Github Pages')
args = parser.parse_args()

# Cleanup the directory where the doxygen output will be stored
shutil.rmtree('docs/api', ignore_errors=True)

# Script which dynamically searches for the packages that exist in the workspace and that have documentation (and are not excluded from the search)
script_name = 'docs/scripts/get_packages_with_docs.sh'

# Name of the file that will include the name of the packages to include in the documentation
config_file = 'mkdocs.yml'

# Name of the key inside the main mkdocs.yml file where the documentation for each package will be placed
nav_key = 'Packages documentation'

# File with packages to ignore when generating doxygen
ignore_doxygen = 'docs/ignore_packages_doxygen.txt'

# Directory to save the output of the doxygen
output_doxygen_directory = os.getcwd() + '/docs/api/packages'
output_markdown_dox_relative = 'api/markdown'
output_markdown_dox_directory = os.getcwd() + '/docs/' + output_markdown_dox_relative

# Setup the location for the Doxyfile template and specifity the output location for the generated files
doxyfile_location = os.getcwd() + '/docs/scripts/Doxyfile'
output_location = os.getcwd() + '/docs/xml_doxygen'

# Read the packages to ignore when generating the doxygen
doxy_ignore_packages = []
with open(ignore_doxygen, 'r') as fp:
    doxy_ignore_packages = fp.read().split('\n')

# --------------------------------------------------------------------------------
# Perform catkin build to guarantee that all packages are indexed in the workspace
# --------------------------------------------------------------------------------
result = subprocess.run(['cd ..; catkin build'], stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True, shell=True)
print(result.stdout)
print(result.stderr)

# Check if there was any error, and if so, just return the same error code
if result.returncode != 0:
    sys.exit(result.returncode)

# --------------------------------------------------
# Search for packages with docs and mkdocs.yml files
# --------------------------------------------------

# Run the bash script that outputs the directory of the packages with documentation (that are not excluded in the ignore_packages file)
result = subprocess.run(['bash', script_name], stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)

# If there was an error getting the list of packages that contain documentation, then stop here
if result.stderr != '':
    raise Exception('Could not discover packages that contain documentation: ' + str(result.stderr))

# Get the path to the packages that contain documentation (and were not excluded)
packages_with_docs = result.stdout.split('\n')
packages_with_docs.pop(-1)

# Pre-process the path so that our data stays inside each package correctly
packages = []
for package_path in packages_with_docs:
    
    # Split the path of the package
    path_by_sub_strings = package_path.split('/')
    path_by_sub_strings_original = package_path

    # Remove the underbars of the name of the packages (make the documentation look prettier)
    for i in range(len(path_by_sub_strings)):
        path_by_sub_strings[i] = path_by_sub_strings[i].replace("_", " ").title()

    # Remove the empty string that is generated in the first item of the list
    path_by_sub_strings.pop(0)

    print('Documentation detected in: ' + package_path)
    
    # Add the packages according to a sub-folder structure to the packages list
    current_pkg = packages
    for i in range(len(path_by_sub_strings)):
        
        # If we are at in the sub-folders (and not yet in the package name), keep adding the tree to the dictionary
        if i != len(path_by_sub_strings)-1:

            # Check if there already exists a meta-package with the same name in the sub-list of packages
            found_meta_package_with_same_name = False
            for meta_package in current_pkg:
                
                if path_by_sub_strings[i] in meta_package:
                    current_pkg = meta_package[path_by_sub_strings[i]]
                    found_meta_package_with_same_name = True
                    break
            
            # If there was not meta package with the same name in the list, then just create one
            if not found_meta_package_with_same_name:
                current_pkg.append({path_by_sub_strings[i]: []})
                current_pkg = current_pkg[-1][path_by_sub_strings[i]]
        
        # If we have reached the package name, make the value corresponding to the key to the path of the documentation yaml file
        else:

            # Check if we the current package is not ignored by doxygen and actually contains code that conforms to doxygen standard
            dox_skip_package = False

            # Check if the package is in the ignore packages, just skip
            for ignore_package in doxy_ignore_packages:
                if ignore_package != '' and ignore_package in path_by_sub_strings_original:
                    dox_skip_package = True

            # Only add the directory to the manual documentation (docs folder) inside the corresponding package
            # to the documentation
            if dox_skip_package:
                current_pkg.append({path_by_sub_strings[i]: '!include .' + package_path + '/mkdocs.yml'})
            # Otherwise, we must generate the doxygen documentation
            else:
                
                # 1. Create the directory to save the doxygen XML output
                os.makedirs(output_doxygen_directory + path_by_sub_strings_original, exist_ok=True)   
        
                # 2. Run doxygen and generate XML output
                result = subprocess.run(
                    ['( cat ' + doxyfile_location + ' ; echo "OUTPUT_DIRECTORY=' + output_doxygen_directory + path_by_sub_strings_original + '" ) | doxygen - '],
                    stdout=subprocess.PIPE, 
                    stderr=subprocess.PIPE, 
                    text=True,
                    cwd=path_by_sub_strings_original[1:],
                    shell=True)

                # 3. Create the directory if it does not exist for the markdown code documentation
                os.makedirs(output_markdown_dox_directory + path_by_sub_strings_original, exist_ok=True)

                # 4. Generate the markdown files from the XML using doxybook
                result = subprocess.run(
                    ['doxybook2 \
                    --input ' + output_doxygen_directory + path_by_sub_strings_original + '/xml \
                    --output ' + output_markdown_dox_directory + path_by_sub_strings_original + ' \
                    --config docs/scripts/doxybook_conf.json \
                    --config-data \'{"baseUrl": "/farol/api/markdown' + package_path + '/"}\''],
                    stdout=subprocess.PIPE, 
                    stderr=subprocess.PIPE, 
                    text=True,
                    shell=True)
                
                path = output_markdown_dox_relative + package_path

                # Add an empty dictionary to the package
                current_pkg.append({path_by_sub_strings[i]: []})

                # 5. Add the manual documentation from the package that inside contains the docs folder and the mkdocs.yml file
                current_pkg[-1][path_by_sub_strings[i]].append({'Documentation': '!include .' + package_path + '/mkdocs.yml'})

                # 6. Check if class documentation was generated, and if so, add it to mkdocs
                if os.path.exists('docs/' + path + '/Classes') and (len(os.listdir('docs/' + path + '/Classes')) != 0 and len(os.listdir('docs/' + path + '/Classes')) != 1):                
                    current_pkg[-1][path_by_sub_strings[i]].append({'Classes': [path + '/Classes']})

                # 7. Check if namespaces documentation was generated, and if so, add it to mkdocs
                if os.path.exists('docs/' + path + '/Namespaces') and (len(os.listdir('docs/' + path + '/Namespaces')) != 0 and len(os.listdir('docs/' + path + '/Namespaces')) != 1):                
                    current_pkg[-1][path_by_sub_strings[i]].append({'Namespaces': [path + '/Namespaces']})
                
                #8. Check if modules documentation was generated, and if so, add it to mkdocs
                if os.path.exists('docs/' + path + '/Modules') and (len(os.listdir('docs/' + path + '/Modules')) != 0 and len(os.listdir('docs/' + path + '/Modules')) != 1):                
                    current_pkg[-1][path_by_sub_strings[i]].append({'Modules': [path + '/Modules']})

                #9. Check if examples documentation was generated, and if so, add it to mkdocs
                if os.path.exists('docs/' + path + '/Examples') and (len(os.listdir('docs/' + path + '/Examples')) != 0 and len(os.listdir('docs/' + path + '/Examples')) != 1):                
                    current_pkg[-1][path_by_sub_strings[i]].append({'Examples': [path + '/Examples']})
        

# --------------------------------------------------------------------------------------
# Update the main mkdocs.yml file with the list of packages and respective documentation
# --------------------------------------------------------------------------------------

# Create a Yaml parser that preserves the comments and file order
yaml = ruamel.yaml.YAML()

# Open the mkdocs yaml and add the packages that we want our documentation to reference
with open(config_file) as fp:
    data = yaml.load(fp)
    
    # Get the list of items inside the 'nav' yaml tag
    for i, docs in enumerate(data['nav']):

        # Find the 'Packages tag'
        for key, value in docs.items():

            # Add the packages that we want our documentation to reference (according to the ignore_packages file) and the ones discovered by ros
            if key == nav_key:
                docs[key] = packages

# Update the mkdocs yaml file, mainting the comments and order of the yaml data
with open(config_file, "w") as fp:
    yaml.dump(data, fp)

# ------------------------------------------------------
# Build the website using mkdocs if we are not deploying
# ------------------------------------------------------

# Perform only the build command if not deploying to github pages
if args.command != 'deploy':
    result = subprocess.run(
                    ['mkdocs build'],
                    stdout=subprocess.PIPE, 
                    stderr=subprocess.PIPE, 
                    text=True,
                    shell=True)
    print(result.stdout)
    print(result.stderr)

# ----------------------------------
# Deploy the website to github pages
# ----------------------------------

# Perform the deployment
if args.command == 'deploy':

    # Update the 
    result = subprocess.run(
                    ['git fetch origin gh-pages:gh-pages; mkdocs gh-deploy --force --verbose'],
                    stdout=subprocess.PIPE, 
                    stderr=subprocess.PIPE, 
                    text=True,
                    shell=True)
    print(result.stdout)
    print(result.stderr)

# Check if there was any error, and if so, just return the same error code
if result.returncode != 0:
    sys.exit(result.returncode)
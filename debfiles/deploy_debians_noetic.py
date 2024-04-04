#!/usr/bin/env python

import os
import sys
import subprocess
import rospkg
import argparse
import logging
import yaml
import xml.etree.ElementTree as ET
import socket

class bcolors:
    HEADER = '\033[95m'
    OKBLUE = '\033[94m'
    OKCYAN = '\033[96m'
    OKGREEN = '\033[92m'
    WARNING = '\033[93m'
    FAIL = '\033[91m'
    ENDC = '\033[0m'
    BOLD = '\033[1m'
    UNDERLINE = '\033[4m'

def get_list_of_debians(path):
    list_of_deb = []
    for file in os.listdir(path):
        if file.endswith(".deb"):    
            list_of_deb.append(file)
    return list_of_deb

def package_in_apt_mark(pkg_name):
    output = subprocess.getoutput("apt-mark showhold")
    splited = output.split("\n")
    result = pkg_name in splited
    return result

def get_package_name(pkg):
    output = subprocess.getoutput("dpkg-deb --field " + pkg)
    splited = output.split("\n")
    package_name = splited[0].split(": ")[1]
    return package_name

def set_package_on_hold(pkg_name):
    retry = 0
    success = False
    while (retry < 10 and not success):
        if package_in_apt_mark(pkg_name):
            success = True
            break
        else:
            retry += 1
            print( bcolors.OKGREEN + "Setting on hold..." + bcolors.ENDC)
            subprocess.call(["sudo", "apt-mark", "hold", pkg_name])
            print( bcolors.OKCYAN + "Checking again..." + bcolors.ENDC)
    return success

def install_all_debians(path, force_overwrite=False):
    list_of_deb = get_list_of_debians(path)
    list_of_pkgs_to_install = []
    print( bcolors.OKGREEN + "--------> Found " + str(len(list_of_deb)) +" debian(s) to install." + bcolors.ENDC    )
    
    for deb in list_of_deb:
        deb_path = path + "/" + deb
        pkg_name = get_package_name(deb_path)
        list_of_pkgs_to_install.append(pkg_name.split("ros-noetic-")[1].replace("-", "_"))
        print( bcolors.OKGREEN + "Installing debian: " + pkg_name + bcolors.ENDC)
        subprocess.call(["sudo", "dpkg", "-i", "--force-overwrite", deb_path])
        print( bcolors.OKGREEN + "Installing debian: " + pkg_name + " done !" + bcolors.ENDC)
        
        set_on_hold_result = set_package_on_hold(pkg_name)
        if set_on_hold_result: 
            print( bcolors.OKGREEN + "Setting " + pkg_name + " on hold done !" + bcolors.ENDC)
        else:
            print( bcolors.FAIL + "Setting " + pkg_name + " on hold failed." + bcolors.ENDC)

    print( bcolors.OKGREEN + "Installing " + str(len(list_of_deb)) + " debian(s) done !" + bcolors.ENDC)
    return list_of_pkgs_to_install

def get_all_installed_gr_pkgs():
    search_path = "/opt/ros/noetic/share"
    filename = "package.xml"
    ros_packages = []

    # Walking top-down from the root
    for root, dir, files in os.walk(search_path):
        if filename in files:
            ros_packages.append(os.path.join(root, filename))

    gr_pkgs = []
    for pkg in ros_packages:
        with open(pkg) as myfile:
             if 'gr_date' in myfile.read():
                gr_pkgs.append(pkg)
    
    return gr_pkgs

def build_rosinstall():
    gr_pkgs = get_all_installed_gr_pkgs()
    # print "gr_pkgs are: " + str(gr_pkgs)

    pkg_infos = get_pkg_infos(gr_pkgs)
    pkg_infos = remove_duplicates(pkg_infos)

    hostname = socket.gethostname()
    filename = hostname + '_gr_pkgs.rosinstall'
    try :
        with open(filename, 'w+') as file:
            for pkg_info in pkg_infos:
                info = {}
                git_info = {}
                git_info["local-name"] = pkg_info["name"]
                git_info["uri"] = pkg_info["gr_url"]
                git_info["version"] = pkg_info["gr_commit"]
                info["- git"] = git_info
                yaml.dump(info, file, indent=4, default_flow_style=False)

        # using sed, we replace all '- git' occurence by - sed (with no quotes !)
        os.system("sed -i \"s/\'- git\'/- git/g\" " + filename)
        print(bcolors.OKGREEN + "Successfully created " + filename + bcolors.ENDC)
    except Exception as err: 
        print(bcolors.FAIL + "Can't build rosinstall file " + filename + " error is: " + str(err) + bcolors.ENDC)

def get_pkg_infos(gr_pkgs):
    pkg_infos = []
    for package_xml_path in gr_pkgs:
        gr_date, gr_branch, gr_commit, gr_url = read_xml(package_xml_path)
        pkg_info = {}
        pkg_split = package_xml_path.split("/")
        pkg_info["path"] = "/".join(pkg_split[0:6])
        pkg_info["name"] = None
        if gr_url is not None:
            pkg_info["name"] = gr_url.split("/")[-1].replace(".git", "")
        pkg_info["gr_date"] = gr_date
        pkg_info["gr_branch"] = gr_branch
        pkg_info["gr_commit"] = gr_commit
        pkg_info["gr_url"] = gr_url
        # print "pkg_info: " + str(pkg_info)
        pkg_infos.append(pkg_info)
    return pkg_infos

def remove_duplicates(pkg_infos):
    cleaned_pkg_infos = []
    list_of_urls = []
    for pkg_info in pkg_infos:
        if pkg_info["gr_url"] not in list_of_urls:
            cleaned_pkg_infos.append(pkg_info)
            list_of_urls.append(pkg_info["gr_url"])

    return cleaned_pkg_infos

def read_xml(package_xml_path):
    try :
        #open file
        tree = ET.parse(package_xml_path)
        root = tree.getroot()
        #check if export balise is present and if not, create it
        if not root.findall("export") : 
            # export_elem = ET.Element('export')
            ET.SubElement(root,"export")
        #get
        export_elem=root.find("export")

        if export_elem.find("gr_date") is None: 
            print (bcolors.FAIL + "gr_date key not found in " + package_xml_path + bcolors.ENDC)
            gr_date = None
        else: 
            gr_date = export_elem.find("gr_date").text
        
        if export_elem.find("gr_branch") is None: 
            print (bcolors.FAIL + "gr_branch key not found in " + package_xml_path + bcolors.ENDC)
            gr_branch = None
        else:
            gr_branch = export_elem.find("gr_branch").text
        
        if export_elem.find("gr_commit") is None: 
            print (bcolors.FAIL + "gr_commit key not found in " + package_xml_path + bcolors.ENDC)
            gr_commit = None
        else:
            gr_commit = export_elem.find("gr_commit").text

        if export_elem.find("gr_url") is None: 
            print (bcolors.FAIL + "gr_url key not found in " + package_xml_path + bcolors.ENDC)
            gr_url = None
        else:
            gr_url = export_elem.find("gr_url").text

        return gr_date, gr_branch, gr_commit, gr_url
    except : 
        print (bcolors.FAIL + "Can't read " + package_xml_path + " , skipping" + bcolors.ENDC)
        return None, None, None, None

if __name__=="__main__":

    if os.geteuid() != 0:
        print(bcolors.FAIL + "You need to have root privileges to run this script.\nPlease try again, this time using 'sudo'. Exiting." + bcolors.ENDC)
        sys.exit(1)

    if not( 1 < len(sys.argv) and len(sys.argv) <= 3):
        print(bcolors.FAIL + "Usage:",sys.argv[0],"<path> force_overwrite[Bool]" + bcolors.ENDC)
        print(bcolors.FAIL + "Illegal number of parameters. Take only one or two argument : the path containing all debian packages to install and if you want to overwrite all debians." + bcolors.ENDC)
        sys.exit(1)

    current_path = os.getcwd()
    relative_path = sys.argv[1]
    force_overwrite = False
    if (len(sys.argv) == 3):
        force_overwrite = sys.argv[2]

    abspath = ""
    if os.path.exists(relative_path):
        abspath = os.path.abspath(relative_path)
    else:
        print(bcolors.FAIL + "Cannot find " + relative_path + bcolors.ENDC)
        exit(1)

    list_of_pkgs_to_install = install_all_debians(abspath, force_overwrite)
    build_rosinstall()
    gr_pkgs = get_all_installed_gr_pkgs()

    if gr_pkgs is not None:
        gr_installed_name_pkgs = [pkg.split("/")[5] for pkg in gr_pkgs]

    print(bcolors.OKGREEN + "--------> Number of GR pkgs to install: " + str(len(list_of_pkgs_to_install)) + bcolors.ENDC)
    print(bcolors.OKGREEN + "--------> Number of GR pkgs currently installed in /opt/ros/noetic/share: " + str(len(gr_installed_name_pkgs)) + bcolors.ENDC)

    for pkgs in list_of_pkgs_to_install:
        if not pkgs in gr_installed_name_pkgs:
            print(bcolors.FAIL + "Check: " + pkgs + " is not installed, it should be installed but might also be ignored" + bcolors.ENDC)

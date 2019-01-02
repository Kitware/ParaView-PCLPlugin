# Introduction

This plugin is brings the full functionality of the [Point Cloud Library (PCL)](http://www.pointclouds.org/) to ParaView in an extensible class hierarchy. Templates and BOOST's preprocessing library are used to generate conversion functions to handle all of PCL's point types with XYZ data. All PCL point clouds with spatial data can thus be converted to and from VTK PolyData instances losslessly, which allows the user to create full PCL workflows within ParaView.

# Installation

Dependencies are listed in the table below along with the version used during development and testing. Minimum required versions have not been determined yet.

| Dependency  | Tested Version  | Minimum Version  |
| :---------: | :-------------: | :--------------: |
| ParaView    | 5.6.0           | ?                |
| PCL         | 1.9             | ?                |
| Eigen       | 3.6.3           | ?                |
| Boost       | 1.68            | ?                |

## PCL Configuration

If PCL is compiled against VTK, segfaults may occur during plugin loading. Strangely, the error only appears during manual loading. To prevent this, either replace `find_package(VTK)` with `find_package(ParaView)` in PCL's CMakeLists.txt file or pass the following options to CMake when building PCL:

* `-DWITH_VTK:BOOL=ON`
* `-DBUILD_visualization:BOOL=OFF`

To enable OpenNI support, PCL must also be compiled with `-DWITH_OPENNI:BOOL=ON` or `-DWITH_OPENNI2:BOOL=ON` and the corresponding OpenNI library must be installed.

### Segfault Details For Reference
~~~~~
*** Process received signal ***
Signal: Segmentation fault (11)
Signal code: Address not mapped (1)
Failing at address: 0x1508
[ 0] /usr/lib/libc.so.6(+0x37e00)[0x7efe8b3cce00]
[ 1] /usr/lib/libvtkCommonSystem-pv5.6.so.1(_ZN11vtkTimerLog17MarkEventInternalEPKcN16vtkTimerLogEntry12LogEntryTypeEPS2_+0x1ea)[0x7efe87ed10aa]
[ 2] /usr/lib/libvtkCommonSystem-pv5.6.so.1(_ZN11vtkTimerLog14MarkStartEventEPKc+0x21)[0x7efe87ed1281]
[ 3] /usr/lib/libvtkPVServerManagerCore-pv5.6.so.1(_ZN15vtkSMOutputPort18GetDataInformationEv+0x214)[0x7efe8b930f24]
[ 4] /usr/lib/libvtkPVServerManagerCore-pv5.6.so.1(_ZN11vtkSMDomain23GetInputDataInformationEPKci+0x80)[0x7efe8b90b8c0]
[ 5] /usr/lib/libvtkPVServerManagerCore-pv5.6.so.1(_ZN20vtkSMArrayListDomain6UpdateEP13vtkSMProperty+0x56)[0x7efe8b8edfa6]
[ 6] /usr/lib/libvtkPVServerManagerCore-pv5.6.so.1(_ZN13vtkSMProperty13UpdateDomainsEv+0x49)[0x7efe8b93ae69]
[ 7] /usr/lib/libvtkPVServerManagerCore-pv5.6.so.1(_ZN9vtkObject22vtkClassMemberCallbackI13vtkSMPropertyEclEPS_mPv+0x3b)[0x7efe8b94040b]
[ 8] /usr/lib/libvtkCommonCore-pv5.6.so.1(+0x325b0a)[0x7efe88e79b0a]
[ 9] /usr/lib/libvtkPVServerManagerCore-pv5.6.so.1(_ZN18vtkSMProxyProperty9LoadStateEP15vtkPVXMLElementP17vtkSMProxyLocator+0x2a9)[0x7efe8b969989]
[10] /usr/lib/libvtkPVServerManagerCore-pv5.6.so.1(_ZN10vtkSMProxy12LoadXMLStateEP15vtkPVXMLElementP17vtkSMProxyLocator+0x14c)[0x7efe8b95613c]
[11] /usr/lib/libvtkPVServerManagerCore-pv5.6.so.1(_ZN20vtkSMDeserializerXML8NewProxyEjP17vtkSMProxyLocator+0x115)[0x7efe8b9096c5]
[12] /usr/lib/libvtkPVServerManagerCore-pv5.6.so.1(_ZN17vtkSMProxyLocator11LocateProxyEj+0x1af)[0x7efe8b96471f]
[13] /usr/lib/libvtkPVServerManagerCore-pv5.6.so.1(_ZN24vtkSMCompoundSourceProxy17ReadXMLAttributesEP24vtkSMSessionProxyManagerP15vtkPVXMLElement+0x27b)[0x7efe8b8ff51b]
[14] /usr/lib/libvtkPVServerManagerCore-pv5.6.so.1(_ZN24vtkSMSessionProxyManager8NewProxyEP15vtkPVXMLElementPKcS3_S3_+0x2c7)[0x7efe8b9865a7]
[15] /usr/lib/libvtkPVServerManagerCore-pv5.6.so.1(_ZN24vtkSMSessionProxyManager8NewProxyEPKcS1_S1_+0xb6)[0x7efe8b986a06]
[16] /usr/lib/libvtkPVServerManagerCore-pv5.6.so.1(_ZN24vtkSMSessionProxyManager17GetPrototypeProxyEPKcS1_+0xf4)[0x7efe8b987f74]
[17] /usr/lib/libvtkpqApplicationComponents-pv5.6.so.1(_ZN23pqProxyGroupMenuManager9getActionERK7QStringS2_+0x19d)[0x7efe8d3ceeed]
[18] /usr/lib/libvtkpqApplicationComponents-pv5.6.so.1(_ZN23pqProxyGroupMenuManager12populateMenuEv+0x3c4)[0x7efe8d3d1254]
[19] /usr/lib/libvtkpqApplicationComponents-pv5.6.so.1(_ZN23pqProxyGroupMenuManager21lookForNewDefinitionsEv+0x16b)[0x7efe8d3d221b]
[20] /usr/lib/libvtkpqApplicationComponents-pv5.6.so.1(_ZN9vtkObject22vtkClassMemberCallbackI23pqProxyGroupMenuManagerEclEPS_mPv+0x3b)[0x7efe8d3d388b]
[21] /usr/lib/libvtkCommonCore-pv5.6.so.1(+0x325b0a)[0x7efe88e79b0a]
[22] /usr/lib/libvtkCommonCore-pv5.6.so.1(+0x325c32)[0x7efe88e79c32]
[23] /usr/lib/libvtkPVServerManagerCore-pv5.6.so.1(_ZN24vtkSMSessionProxyManager12ExecuteEventEP9vtkObjectmPv+0x15f)[0x7efe8b98553f]
[24] /usr/lib/libvtkCommonCore-pv5.6.so.1(+0x325c32)[0x7efe88e79c32]
[25] /usr/lib/libvtkCommonCore-pv5.6.so.1(+0x325c32)[0x7efe88e79c32]
[26] /usr/lib/libvtkPVServerImplementationCore-pv5.6.so.1(_ZN27vtkSIProxyDefinitionManager20LoadConfigurationXMLEP15vtkPVXMLElementb+0x2f1)[0x7efe899dacb1]
[27] /usr/lib/libvtkPVServerImplementationCore-pv5.6.so.1(_ZN27vtkSIProxyDefinitionManager30LoadConfigurationXMLFromStringEPKcb+0x6c)[0x7efe899dad7c]
[28] /usr/lib/libvtkPVServerImplementationCore-pv5.6.so.1(_ZN27vtkSIProxyDefinitionManager12HandlePluginEP11vtkPVPlugin+0xdb)[0x7efe899dae7b]
[29] /usr/lib/libvtkPVServerImplementationCore-pv5.6.so.1(_ZN9vtkObject22vtkClassMemberCallbackI27vtkSIProxyDefinitionManagerEclEPS_mPv+0x69)[0x7efe899e09b9]
*** End of error message ***
~~~~~


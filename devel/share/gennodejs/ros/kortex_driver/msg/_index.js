
"use strict";

let ApiOptions = require('./ApiOptions.js');
let KortexError = require('./KortexError.js');
let SubErrorCodes = require('./SubErrorCodes.js');
let ErrorCodes = require('./ErrorCodes.js');
let ActuatorConfig_SafetyLimitType = require('./ActuatorConfig_SafetyLimitType.js');
let ActuatorConfig_ServiceVersion = require('./ActuatorConfig_ServiceVersion.js');
let EncoderDerivativeParameters = require('./EncoderDerivativeParameters.js');
let PositionCommand = require('./PositionCommand.js');
let ControlLoopSelection = require('./ControlLoopSelection.js');
let CommandMode = require('./CommandMode.js');
let Servoing = require('./Servoing.js');
let ActuatorConfig_ControlMode = require('./ActuatorConfig_ControlMode.js');
let LoopSelection = require('./LoopSelection.js');
let CustomDataSelection = require('./CustomDataSelection.js');
let FrequencyResponse = require('./FrequencyResponse.js');
let RampResponse = require('./RampResponse.js');
let ActuatorConfig_ControlModeInformation = require('./ActuatorConfig_ControlModeInformation.js');
let SafetyIdentifierBankA = require('./SafetyIdentifierBankA.js');
let ControlLoop = require('./ControlLoop.js');
let AxisPosition = require('./AxisPosition.js');
let CoggingFeedforwardMode = require('./CoggingFeedforwardMode.js');
let CustomDataIndex = require('./CustomDataIndex.js');
let CoggingFeedforwardModeInformation = require('./CoggingFeedforwardModeInformation.js');
let AxisOffsets = require('./AxisOffsets.js');
let CommandModeInformation = require('./CommandModeInformation.js');
let ControlLoopParameters = require('./ControlLoopParameters.js');
let TorqueCalibration = require('./TorqueCalibration.js');
let StepResponse = require('./StepResponse.js');
let VectorDriveParameters = require('./VectorDriveParameters.js');
let TorqueOffset = require('./TorqueOffset.js');
let ActuatorCyclic_Command = require('./ActuatorCyclic_Command.js');
let ActuatorCyclic_CustomData = require('./ActuatorCyclic_CustomData.js');
let StatusFlags = require('./StatusFlags.js');
let CommandFlags = require('./CommandFlags.js');
let ActuatorCyclic_ServiceVersion = require('./ActuatorCyclic_ServiceVersion.js');
let ActuatorCyclic_MessageId = require('./ActuatorCyclic_MessageId.js');
let ActuatorCyclic_Feedback = require('./ActuatorCyclic_Feedback.js');
let TrajectoryErrorType = require('./TrajectoryErrorType.js');
let SafetyEvent = require('./SafetyEvent.js');
let Base_Position = require('./Base_Position.js');
let ControllerConfigurationList = require('./ControllerConfigurationList.js');
let Base_RotationMatrix = require('./Base_RotationMatrix.js');
let ProtectionZoneEvent = require('./ProtectionZoneEvent.js');
let CartesianLimitation = require('./CartesianLimitation.js');
let GpioBehavior = require('./GpioBehavior.js');
let UserProfileList = require('./UserProfileList.js');
let BridgeType = require('./BridgeType.js');
let SignalQuality = require('./SignalQuality.js');
let TrajectoryInfoType = require('./TrajectoryInfoType.js');
let GripperRequest = require('./GripperRequest.js');
let ControlModeNotificationList = require('./ControlModeNotificationList.js');
let BridgeConfig = require('./BridgeConfig.js');
let MapHandle = require('./MapHandle.js');
let Map = require('./Map.js');
let WifiConfigurationList = require('./WifiConfigurationList.js');
let Timeout = require('./Timeout.js');
let ActivateMapHandle = require('./ActivateMapHandle.js');
let ProtectionZoneHandle = require('./ProtectionZoneHandle.js');
let ControllerElementHandle_identifier = require('./ControllerElementHandle_identifier.js');
let MapElement = require('./MapElement.js');
let Sequence = require('./Sequence.js');
let BridgeIdentifier = require('./BridgeIdentifier.js');
let JointTorque = require('./JointTorque.js');
let ControllerElementHandle = require('./ControllerElementHandle.js');
let Base_JointSpeeds = require('./Base_JointSpeeds.js');
let UserNotificationList = require('./UserNotificationList.js');
let JointsLimitationsList = require('./JointsLimitationsList.js');
let SequenceTask = require('./SequenceTask.js');
let CartesianTrajectoryConstraint_type = require('./CartesianTrajectoryConstraint_type.js');
let RobotEventNotification = require('./RobotEventNotification.js');
let TransformationRow = require('./TransformationRow.js');
let ConstrainedJointAngles = require('./ConstrainedJointAngles.js');
let Base_ControlModeNotification = require('./Base_ControlModeNotification.js');
let SequenceInfoNotificationList = require('./SequenceInfoNotificationList.js');
let MapList = require('./MapList.js');
let FactoryEvent = require('./FactoryEvent.js');
let Wrench = require('./Wrench.js');
let BridgeStatus = require('./BridgeStatus.js');
let MapEvent_events = require('./MapEvent_events.js');
let WifiConfiguration = require('./WifiConfiguration.js');
let CartesianTrajectoryConstraint = require('./CartesianTrajectoryConstraint.js');
let JointTrajectoryConstraintType = require('./JointTrajectoryConstraintType.js');
let ControllerElementEventType = require('./ControllerElementEventType.js');
let TwistCommand = require('./TwistCommand.js');
let ActionEvent = require('./ActionEvent.js');
let ControllerState = require('./ControllerState.js');
let CartesianSpeed = require('./CartesianSpeed.js');
let NetworkEvent = require('./NetworkEvent.js');
let SequenceTaskConfiguration = require('./SequenceTaskConfiguration.js');
let SystemTime = require('./SystemTime.js');
let WrenchLimitation = require('./WrenchLimitation.js');
let Action_action_parameters = require('./Action_action_parameters.js');
let ProtectionZoneNotification = require('./ProtectionZoneNotification.js');
let UserNotification = require('./UserNotification.js');
let GripperMode = require('./GripperMode.js');
let MappingList = require('./MappingList.js');
let Twist = require('./Twist.js');
let JointAngle = require('./JointAngle.js');
let WrenchCommand = require('./WrenchCommand.js');
let WifiEncryptionType = require('./WifiEncryptionType.js');
let SequenceInfoNotification = require('./SequenceInfoNotification.js');
let Waypoint = require('./Waypoint.js');
let OperatingModeNotification = require('./OperatingModeNotification.js');
let ProtectionZone = require('./ProtectionZone.js');
let CommunicationInterfaceConfiguration = require('./CommunicationInterfaceConfiguration.js');
let RobotEventNotificationList = require('./RobotEventNotificationList.js');
let NetworkHandle = require('./NetworkHandle.js');
let ActionNotification = require('./ActionNotification.js');
let ServoingModeInformation = require('./ServoingModeInformation.js');
let ActionType = require('./ActionType.js');
let JointLimitation = require('./JointLimitation.js');
let Delay = require('./Delay.js');
let CartesianWaypoint = require('./CartesianWaypoint.js');
let SequenceHandle = require('./SequenceHandle.js');
let Gen3GpioPinId = require('./Gen3GpioPinId.js');
let Action = require('./Action.js');
let ControllerNotificationList = require('./ControllerNotificationList.js');
let Base_ControlModeInformation = require('./Base_ControlModeInformation.js');
let ShapeType = require('./ShapeType.js');
let MapEvent = require('./MapEvent.js');
let WrenchMode = require('./WrenchMode.js');
let ServoingMode = require('./ServoingMode.js');
let JointTrajectoryConstraint = require('./JointTrajectoryConstraint.js');
let ControllerList = require('./ControllerList.js');
let MappingInfoNotificationList = require('./MappingInfoNotificationList.js');
let BackupEvent = require('./BackupEvent.js');
let ChangeWrench = require('./ChangeWrench.js');
let IPv4Configuration = require('./IPv4Configuration.js');
let MapGroupList = require('./MapGroupList.js');
let ControllerNotification = require('./ControllerNotification.js');
let Base_ControlMode = require('./Base_ControlMode.js');
let Mapping = require('./Mapping.js');
let Xbox360DigitalInputIdentifier = require('./Xbox360DigitalInputIdentifier.js');
let SequenceList = require('./SequenceList.js');
let BridgeResult = require('./BridgeResult.js');
let SoundType = require('./SoundType.js');
let FirmwareBundleVersions = require('./FirmwareBundleVersions.js');
let Gripper = require('./Gripper.js');
let ConfigurationChangeNotification_configuration_change = require('./ConfigurationChangeNotification_configuration_change.js');
let EventIdSequenceInfoNotification = require('./EventIdSequenceInfoNotification.js');
let TwistLimitation = require('./TwistLimitation.js');
let RobotEvent = require('./RobotEvent.js');
let Base_ServiceVersion = require('./Base_ServiceVersion.js');
let UserList = require('./UserList.js');
let Base_SafetyIdentifier = require('./Base_SafetyIdentifier.js');
let GpioPinPropertyFlags = require('./GpioPinPropertyFlags.js');
let ArmStateNotification = require('./ArmStateNotification.js');
let TrajectoryErrorIdentifier = require('./TrajectoryErrorIdentifier.js');
let ProtectionZoneInformation = require('./ProtectionZoneInformation.js');
let TrajectoryContinuityMode = require('./TrajectoryContinuityMode.js');
let Pose = require('./Pose.js');
let BridgePortConfig = require('./BridgePortConfig.js');
let ActionHandle = require('./ActionHandle.js');
let ActionList = require('./ActionList.js');
let WristDigitalInputIdentifier = require('./WristDigitalInputIdentifier.js');
let Snapshot = require('./Snapshot.js');
let SequenceInformation = require('./SequenceInformation.js');
let TrajectoryInfo = require('./TrajectoryInfo.js');
let ControllerEvent = require('./ControllerEvent.js');
let FactoryNotification = require('./FactoryNotification.js');
let MapGroup = require('./MapGroup.js');
let ArmStateInformation = require('./ArmStateInformation.js');
let FullIPv4Configuration = require('./FullIPv4Configuration.js');
let WifiInformationList = require('./WifiInformationList.js');
let Base_RotationMatrixRow = require('./Base_RotationMatrixRow.js');
let ProtectionZoneList = require('./ProtectionZoneList.js');
let AdvancedSequenceHandle = require('./AdvancedSequenceHandle.js');
let SequenceTaskHandle = require('./SequenceTaskHandle.js');
let LedState = require('./LedState.js');
let NetworkNotificationList = require('./NetworkNotificationList.js');
let GpioAction = require('./GpioAction.js');
let PasswordChange = require('./PasswordChange.js');
let UserProfile = require('./UserProfile.js');
let ControllerType = require('./ControllerType.js');
let WifiSecurityType = require('./WifiSecurityType.js');
let SafetyNotificationList = require('./SafetyNotificationList.js');
let ChangeJointSpeeds = require('./ChangeJointSpeeds.js');
let JointTorques = require('./JointTorques.js');
let NavigationDirection = require('./NavigationDirection.js');
let Orientation = require('./Orientation.js');
let SnapshotType = require('./SnapshotType.js');
let Base_GpioConfiguration = require('./Base_GpioConfiguration.js');
let GpioEvent = require('./GpioEvent.js');
let LimitationType = require('./LimitationType.js');
let KinematicTrajectoryConstraints = require('./KinematicTrajectoryConstraints.js');
let ActionNotificationList = require('./ActionNotificationList.js');
let OperatingMode = require('./OperatingMode.js');
let TransformationMatrix = require('./TransformationMatrix.js');
let MapGroupHandle = require('./MapGroupHandle.js');
let PreComputedJointTrajectoryElement = require('./PreComputedJointTrajectoryElement.js');
let GpioConfigurationList = require('./GpioConfigurationList.js');
let ControllerInputType = require('./ControllerInputType.js');
let ControllerBehavior = require('./ControllerBehavior.js');
let Xbox360AnalogInputIdentifier = require('./Xbox360AnalogInputIdentifier.js');
let ChangeTwist = require('./ChangeTwist.js');
let AppendActionInformation = require('./AppendActionInformation.js');
let WaypointList = require('./WaypointList.js');
let ConstrainedPose = require('./ConstrainedPose.js');
let ServoingModeNotification = require('./ServoingModeNotification.js');
let IKData = require('./IKData.js');
let WifiInformation = require('./WifiInformation.js');
let Base_CapSenseConfig = require('./Base_CapSenseConfig.js');
let GpioCommand = require('./GpioCommand.js');
let SequenceTasksPair = require('./SequenceTasksPair.js');
let Finger = require('./Finger.js');
let JointNavigationDirection = require('./JointNavigationDirection.js');
let FirmwareComponentVersion = require('./FirmwareComponentVersion.js');
let Base_Stop = require('./Base_Stop.js');
let AdmittanceMode = require('./AdmittanceMode.js');
let ConfigurationNotificationEvent = require('./ConfigurationNotificationEvent.js');
let ConstrainedJointAngle = require('./ConstrainedJointAngle.js');
let TrajectoryErrorReport = require('./TrajectoryErrorReport.js');
let ActuatorInformation = require('./ActuatorInformation.js');
let AngularWaypoint = require('./AngularWaypoint.js');
let ControllerHandle = require('./ControllerHandle.js');
let GripperCommand = require('./GripperCommand.js');
let SequenceTasks = require('./SequenceTasks.js');
let ConstrainedPosition = require('./ConstrainedPosition.js');
let IPv4Information = require('./IPv4Information.js');
let ControllerConfigurationMode = require('./ControllerConfigurationMode.js');
let ActionExecutionState = require('./ActionExecutionState.js');
let RequestedActionType = require('./RequestedActionType.js');
let ConstrainedOrientation = require('./ConstrainedOrientation.js');
let Faults = require('./Faults.js');
let JointAngles = require('./JointAngles.js');
let NetworkType = require('./NetworkType.js');
let Query = require('./Query.js');
let ConfigurationChangeNotification = require('./ConfigurationChangeNotification.js');
let Admittance = require('./Admittance.js');
let ProtectionZoneNotificationList = require('./ProtectionZoneNotificationList.js');
let MappingHandle = require('./MappingHandle.js');
let Ssid = require('./Ssid.js');
let SequenceTasksConfiguration = require('./SequenceTasksConfiguration.js');
let ControllerNotification_state = require('./ControllerNotification_state.js');
let JointSpeed = require('./JointSpeed.js');
let OperatingModeInformation = require('./OperatingModeInformation.js');
let Base_CapSenseMode = require('./Base_CapSenseMode.js');
let EmergencyStop = require('./EmergencyStop.js');
let UserEvent = require('./UserEvent.js');
let WaypointValidationReport = require('./WaypointValidationReport.js');
let PreComputedJointTrajectory = require('./PreComputedJointTrajectory.js');
let BridgeList = require('./BridgeList.js');
let ControllerElementState = require('./ControllerElementState.js');
let Point = require('./Point.js');
let SequenceTasksRange = require('./SequenceTasksRange.js');
let FullUserProfile = require('./FullUserProfile.js');
let ConfigurationChangeNotificationList = require('./ConfigurationChangeNotificationList.js');
let TrajectoryErrorElement = require('./TrajectoryErrorElement.js');
let ServoingModeNotificationList = require('./ServoingModeNotificationList.js');
let CartesianLimitationList = require('./CartesianLimitationList.js');
let MappingInfoNotification = require('./MappingInfoNotification.js');
let NetworkNotification = require('./NetworkNotification.js');
let Waypoint_type_of_waypoint = require('./Waypoint_type_of_waypoint.js');
let ControllerConfiguration = require('./ControllerConfiguration.js');
let SwitchControlMapping = require('./SwitchControlMapping.js');
let OperatingModeNotificationList = require('./OperatingModeNotificationList.js');
let GpioPinConfiguration = require('./GpioPinConfiguration.js');
let ControllerEventType = require('./ControllerEventType.js');
let ZoneShape = require('./ZoneShape.js');
let BaseCyclic_ServiceVersion = require('./BaseCyclic_ServiceVersion.js');
let BaseCyclic_CustomData = require('./BaseCyclic_CustomData.js');
let ActuatorCommand = require('./ActuatorCommand.js');
let ActuatorFeedback = require('./ActuatorFeedback.js');
let BaseCyclic_Command = require('./BaseCyclic_Command.js');
let BaseCyclic_Feedback = require('./BaseCyclic_Feedback.js');
let ActuatorCustomData = require('./ActuatorCustomData.js');
let BaseFeedback = require('./BaseFeedback.js');
let UARTParity = require('./UARTParity.js');
let Empty = require('./Empty.js');
let UARTDeviceIdentification = require('./UARTDeviceIdentification.js');
let Unit = require('./Unit.js');
let CountryCode = require('./CountryCode.js');
let DeviceTypes = require('./DeviceTypes.js');
let UARTStopBits = require('./UARTStopBits.js');
let ArmState = require('./ArmState.js');
let Connection = require('./Connection.js');
let UARTSpeed = require('./UARTSpeed.js');
let UARTWordLength = require('./UARTWordLength.js');
let NotificationOptions = require('./NotificationOptions.js');
let UARTConfiguration = require('./UARTConfiguration.js');
let DeviceHandle = require('./DeviceHandle.js');
let NotificationType = require('./NotificationType.js');
let Permission = require('./Permission.js');
let SafetyNotification = require('./SafetyNotification.js');
let SafetyHandle = require('./SafetyHandle.js');
let CountryCodeIdentifier = require('./CountryCodeIdentifier.js');
let Timestamp = require('./Timestamp.js');
let CartesianReferenceFrame = require('./CartesianReferenceFrame.js');
let NotificationHandle = require('./NotificationHandle.js');
let UserProfileHandle = require('./UserProfileHandle.js');
let SafetyStatusValue = require('./SafetyStatusValue.js');
let JointAccelerationSoftLimits = require('./JointAccelerationSoftLimits.js');
let DesiredSpeeds = require('./DesiredSpeeds.js');
let GravityVector = require('./GravityVector.js');
let ControlConfigurationEvent = require('./ControlConfigurationEvent.js');
let TwistLinearSoftLimit = require('./TwistLinearSoftLimit.js');
let ControlConfig_JointSpeeds = require('./ControlConfig_JointSpeeds.js');
let KinematicLimits = require('./KinematicLimits.js');
let LinearTwist = require('./LinearTwist.js');
let CartesianTransform = require('./CartesianTransform.js');
let JointSpeedSoftLimits = require('./JointSpeedSoftLimits.js');
let ControlConfig_ServiceVersion = require('./ControlConfig_ServiceVersion.js');
let TwistAngularSoftLimit = require('./TwistAngularSoftLimit.js');
let ControlConfig_ControlMode = require('./ControlConfig_ControlMode.js');
let CartesianReferenceFrameInfo = require('./CartesianReferenceFrameInfo.js');
let ToolConfiguration = require('./ToolConfiguration.js');
let AngularTwist = require('./AngularTwist.js');
let ControlConfig_ControlModeInformation = require('./ControlConfig_ControlModeInformation.js');
let KinematicLimitsList = require('./KinematicLimitsList.js');
let ControlConfig_Position = require('./ControlConfig_Position.js');
let PayloadInformation = require('./PayloadInformation.js');
let ControlConfigurationNotification = require('./ControlConfigurationNotification.js');
let ControlConfig_ControlModeNotification = require('./ControlConfig_ControlModeNotification.js');
let CalibrationItem = require('./CalibrationItem.js');
let SafetyConfigurationList = require('./SafetyConfigurationList.js');
let BootloaderVersion = require('./BootloaderVersion.js');
let SafetyStatus = require('./SafetyStatus.js');
let SerialNumber = require('./SerialNumber.js');
let Calibration = require('./Calibration.js');
let DeviceConfig_ServiceVersion = require('./DeviceConfig_ServiceVersion.js');
let CalibrationParameter = require('./CalibrationParameter.js');
let MACAddress = require('./MACAddress.js');
let PartNumber = require('./PartNumber.js');
let PartNumberRevision = require('./PartNumberRevision.js');
let SafetyEnable = require('./SafetyEnable.js');
let FirmwareVersion = require('./FirmwareVersion.js');
let DeviceType = require('./DeviceType.js');
let CalibrationParameter_value = require('./CalibrationParameter_value.js');
let CalibrationElement = require('./CalibrationElement.js');
let DeviceConfig_CapSenseConfig = require('./DeviceConfig_CapSenseConfig.js');
let SafetyConfiguration = require('./SafetyConfiguration.js');
let DeviceConfig_CapSenseMode = require('./DeviceConfig_CapSenseMode.js');
let SafetyThreshold = require('./SafetyThreshold.js');
let DeviceConfig_SafetyLimitType = require('./DeviceConfig_SafetyLimitType.js');
let CalibrationStatus = require('./CalibrationStatus.js');
let CalibrationResult = require('./CalibrationResult.js');
let IPv4Settings = require('./IPv4Settings.js');
let RunModes = require('./RunModes.js');
let SafetyInformationList = require('./SafetyInformationList.js');
let RunMode = require('./RunMode.js');
let PowerOnSelfTestResult = require('./PowerOnSelfTestResult.js');
let SafetyInformation = require('./SafetyInformation.js');
let CapSenseRegister = require('./CapSenseRegister.js');
let RebootRqst = require('./RebootRqst.js');
let ModelNumber = require('./ModelNumber.js');
let DeviceHandles = require('./DeviceHandles.js');
let DeviceManager_ServiceVersion = require('./DeviceManager_ServiceVersion.js');
let RobotiqGripperStatusFlags = require('./RobotiqGripperStatusFlags.js');
let GripperConfig_SafetyIdentifier = require('./GripperConfig_SafetyIdentifier.js');
let GripperCyclic_MessageId = require('./GripperCyclic_MessageId.js');
let CustomDataUnit = require('./CustomDataUnit.js');
let GripperCyclic_ServiceVersion = require('./GripperCyclic_ServiceVersion.js');
let GripperCyclic_Command = require('./GripperCyclic_Command.js');
let MotorCommand = require('./MotorCommand.js');
let GripperCyclic_CustomData = require('./GripperCyclic_CustomData.js');
let GripperCyclic_Feedback = require('./GripperCyclic_Feedback.js');
let MotorFeedback = require('./MotorFeedback.js');
let GPIOState = require('./GPIOState.js');
let GPIOIdentification = require('./GPIOIdentification.js');
let I2CWriteRegisterParameter = require('./I2CWriteRegisterParameter.js');
let UARTPortId = require('./UARTPortId.js');
let I2CReadRegisterParameter = require('./I2CReadRegisterParameter.js');
let EthernetDeviceIdentification = require('./EthernetDeviceIdentification.js');
let InterconnectConfig_GPIOConfiguration = require('./InterconnectConfig_GPIOConfiguration.js');
let I2CData = require('./I2CData.js');
let InterconnectConfig_SafetyIdentifier = require('./InterconnectConfig_SafetyIdentifier.js');
let I2CRegisterAddressSize = require('./I2CRegisterAddressSize.js');
let EthernetDuplex = require('./EthernetDuplex.js');
let I2CDeviceIdentification = require('./I2CDeviceIdentification.js');
let I2CWriteParameter = require('./I2CWriteParameter.js');
let EthernetConfiguration = require('./EthernetConfiguration.js');
let I2CMode = require('./I2CMode.js');
let I2CConfiguration = require('./I2CConfiguration.js');
let EthernetSpeed = require('./EthernetSpeed.js');
let EthernetDevice = require('./EthernetDevice.js');
let GPIOMode = require('./GPIOMode.js');
let GPIOIdentifier = require('./GPIOIdentifier.js');
let InterconnectConfig_ServiceVersion = require('./InterconnectConfig_ServiceVersion.js');
let GPIOPull = require('./GPIOPull.js');
let I2CDeviceAddressing = require('./I2CDeviceAddressing.js');
let GPIOValue = require('./GPIOValue.js');
let I2CDevice = require('./I2CDevice.js');
let I2CReadParameter = require('./I2CReadParameter.js');
let InterconnectCyclic_MessageId = require('./InterconnectCyclic_MessageId.js');
let InterconnectCyclic_ServiceVersion = require('./InterconnectCyclic_ServiceVersion.js');
let InterconnectCyclic_Feedback_tool_feedback = require('./InterconnectCyclic_Feedback_tool_feedback.js');
let InterconnectCyclic_Feedback = require('./InterconnectCyclic_Feedback.js');
let InterconnectCyclic_Command = require('./InterconnectCyclic_Command.js');
let InterconnectCyclic_Command_tool_command = require('./InterconnectCyclic_Command_tool_command.js');
let InterconnectCyclic_CustomData = require('./InterconnectCyclic_CustomData.js');
let InterconnectCyclic_CustomData_tool_customData = require('./InterconnectCyclic_CustomData_tool_customData.js');
let ModelId = require('./ModelId.js');
let WristType = require('./WristType.js');
let BaseType = require('./BaseType.js');
let CompleteProductConfiguration = require('./CompleteProductConfiguration.js');
let ProductConfigurationEndEffectorType = require('./ProductConfigurationEndEffectorType.js');
let InterfaceModuleType = require('./InterfaceModuleType.js');
let EndEffectorType = require('./EndEffectorType.js');
let ArmLaterality = require('./ArmLaterality.js');
let VisionModuleType = require('./VisionModuleType.js');
let FocusAction = require('./FocusAction.js');
let ExtrinsicParameters = require('./ExtrinsicParameters.js');
let OptionIdentifier = require('./OptionIdentifier.js');
let BitRate = require('./BitRate.js');
let SensorFocusAction = require('./SensorFocusAction.js');
let IntrinsicProfileIdentifier = require('./IntrinsicProfileIdentifier.js');
let VisionConfig_ServiceVersion = require('./VisionConfig_ServiceVersion.js');
let DistortionCoefficients = require('./DistortionCoefficients.js');
let OptionValue = require('./OptionValue.js');
let Option = require('./Option.js');
let VisionEvent = require('./VisionEvent.js');
let Resolution = require('./Resolution.js');
let SensorIdentifier = require('./SensorIdentifier.js');
let IntrinsicParameters = require('./IntrinsicParameters.js');
let FocusPoint = require('./FocusPoint.js');
let VisionConfig_RotationMatrix = require('./VisionConfig_RotationMatrix.js');
let SensorFocusAction_action_parameters = require('./SensorFocusAction_action_parameters.js');
let VisionConfig_RotationMatrixRow = require('./VisionConfig_RotationMatrixRow.js');
let ManualFocus = require('./ManualFocus.js');
let Sensor = require('./Sensor.js');
let FrameRate = require('./FrameRate.js');
let TranslationVector = require('./TranslationVector.js');
let OptionInformation = require('./OptionInformation.js');
let VisionNotification = require('./VisionNotification.js');
let SensorSettings = require('./SensorSettings.js');
let FollowCartesianTrajectoryAction = require('./FollowCartesianTrajectoryAction.js');
let FollowCartesianTrajectoryActionGoal = require('./FollowCartesianTrajectoryActionGoal.js');
let FollowCartesianTrajectoryActionFeedback = require('./FollowCartesianTrajectoryActionFeedback.js');
let FollowCartesianTrajectoryGoal = require('./FollowCartesianTrajectoryGoal.js');
let FollowCartesianTrajectoryResult = require('./FollowCartesianTrajectoryResult.js');
let FollowCartesianTrajectoryFeedback = require('./FollowCartesianTrajectoryFeedback.js');
let FollowCartesianTrajectoryActionResult = require('./FollowCartesianTrajectoryActionResult.js');

module.exports = {
  ApiOptions: ApiOptions,
  KortexError: KortexError,
  SubErrorCodes: SubErrorCodes,
  ErrorCodes: ErrorCodes,
  ActuatorConfig_SafetyLimitType: ActuatorConfig_SafetyLimitType,
  ActuatorConfig_ServiceVersion: ActuatorConfig_ServiceVersion,
  EncoderDerivativeParameters: EncoderDerivativeParameters,
  PositionCommand: PositionCommand,
  ControlLoopSelection: ControlLoopSelection,
  CommandMode: CommandMode,
  Servoing: Servoing,
  ActuatorConfig_ControlMode: ActuatorConfig_ControlMode,
  LoopSelection: LoopSelection,
  CustomDataSelection: CustomDataSelection,
  FrequencyResponse: FrequencyResponse,
  RampResponse: RampResponse,
  ActuatorConfig_ControlModeInformation: ActuatorConfig_ControlModeInformation,
  SafetyIdentifierBankA: SafetyIdentifierBankA,
  ControlLoop: ControlLoop,
  AxisPosition: AxisPosition,
  CoggingFeedforwardMode: CoggingFeedforwardMode,
  CustomDataIndex: CustomDataIndex,
  CoggingFeedforwardModeInformation: CoggingFeedforwardModeInformation,
  AxisOffsets: AxisOffsets,
  CommandModeInformation: CommandModeInformation,
  ControlLoopParameters: ControlLoopParameters,
  TorqueCalibration: TorqueCalibration,
  StepResponse: StepResponse,
  VectorDriveParameters: VectorDriveParameters,
  TorqueOffset: TorqueOffset,
  ActuatorCyclic_Command: ActuatorCyclic_Command,
  ActuatorCyclic_CustomData: ActuatorCyclic_CustomData,
  StatusFlags: StatusFlags,
  CommandFlags: CommandFlags,
  ActuatorCyclic_ServiceVersion: ActuatorCyclic_ServiceVersion,
  ActuatorCyclic_MessageId: ActuatorCyclic_MessageId,
  ActuatorCyclic_Feedback: ActuatorCyclic_Feedback,
  TrajectoryErrorType: TrajectoryErrorType,
  SafetyEvent: SafetyEvent,
  Base_Position: Base_Position,
  ControllerConfigurationList: ControllerConfigurationList,
  Base_RotationMatrix: Base_RotationMatrix,
  ProtectionZoneEvent: ProtectionZoneEvent,
  CartesianLimitation: CartesianLimitation,
  GpioBehavior: GpioBehavior,
  UserProfileList: UserProfileList,
  BridgeType: BridgeType,
  SignalQuality: SignalQuality,
  TrajectoryInfoType: TrajectoryInfoType,
  GripperRequest: GripperRequest,
  ControlModeNotificationList: ControlModeNotificationList,
  BridgeConfig: BridgeConfig,
  MapHandle: MapHandle,
  Map: Map,
  WifiConfigurationList: WifiConfigurationList,
  Timeout: Timeout,
  ActivateMapHandle: ActivateMapHandle,
  ProtectionZoneHandle: ProtectionZoneHandle,
  ControllerElementHandle_identifier: ControllerElementHandle_identifier,
  MapElement: MapElement,
  Sequence: Sequence,
  BridgeIdentifier: BridgeIdentifier,
  JointTorque: JointTorque,
  ControllerElementHandle: ControllerElementHandle,
  Base_JointSpeeds: Base_JointSpeeds,
  UserNotificationList: UserNotificationList,
  JointsLimitationsList: JointsLimitationsList,
  SequenceTask: SequenceTask,
  CartesianTrajectoryConstraint_type: CartesianTrajectoryConstraint_type,
  RobotEventNotification: RobotEventNotification,
  TransformationRow: TransformationRow,
  ConstrainedJointAngles: ConstrainedJointAngles,
  Base_ControlModeNotification: Base_ControlModeNotification,
  SequenceInfoNotificationList: SequenceInfoNotificationList,
  MapList: MapList,
  FactoryEvent: FactoryEvent,
  Wrench: Wrench,
  BridgeStatus: BridgeStatus,
  MapEvent_events: MapEvent_events,
  WifiConfiguration: WifiConfiguration,
  CartesianTrajectoryConstraint: CartesianTrajectoryConstraint,
  JointTrajectoryConstraintType: JointTrajectoryConstraintType,
  ControllerElementEventType: ControllerElementEventType,
  TwistCommand: TwistCommand,
  ActionEvent: ActionEvent,
  ControllerState: ControllerState,
  CartesianSpeed: CartesianSpeed,
  NetworkEvent: NetworkEvent,
  SequenceTaskConfiguration: SequenceTaskConfiguration,
  SystemTime: SystemTime,
  WrenchLimitation: WrenchLimitation,
  Action_action_parameters: Action_action_parameters,
  ProtectionZoneNotification: ProtectionZoneNotification,
  UserNotification: UserNotification,
  GripperMode: GripperMode,
  MappingList: MappingList,
  Twist: Twist,
  JointAngle: JointAngle,
  WrenchCommand: WrenchCommand,
  WifiEncryptionType: WifiEncryptionType,
  SequenceInfoNotification: SequenceInfoNotification,
  Waypoint: Waypoint,
  OperatingModeNotification: OperatingModeNotification,
  ProtectionZone: ProtectionZone,
  CommunicationInterfaceConfiguration: CommunicationInterfaceConfiguration,
  RobotEventNotificationList: RobotEventNotificationList,
  NetworkHandle: NetworkHandle,
  ActionNotification: ActionNotification,
  ServoingModeInformation: ServoingModeInformation,
  ActionType: ActionType,
  JointLimitation: JointLimitation,
  Delay: Delay,
  CartesianWaypoint: CartesianWaypoint,
  SequenceHandle: SequenceHandle,
  Gen3GpioPinId: Gen3GpioPinId,
  Action: Action,
  ControllerNotificationList: ControllerNotificationList,
  Base_ControlModeInformation: Base_ControlModeInformation,
  ShapeType: ShapeType,
  MapEvent: MapEvent,
  WrenchMode: WrenchMode,
  ServoingMode: ServoingMode,
  JointTrajectoryConstraint: JointTrajectoryConstraint,
  ControllerList: ControllerList,
  MappingInfoNotificationList: MappingInfoNotificationList,
  BackupEvent: BackupEvent,
  ChangeWrench: ChangeWrench,
  IPv4Configuration: IPv4Configuration,
  MapGroupList: MapGroupList,
  ControllerNotification: ControllerNotification,
  Base_ControlMode: Base_ControlMode,
  Mapping: Mapping,
  Xbox360DigitalInputIdentifier: Xbox360DigitalInputIdentifier,
  SequenceList: SequenceList,
  BridgeResult: BridgeResult,
  SoundType: SoundType,
  FirmwareBundleVersions: FirmwareBundleVersions,
  Gripper: Gripper,
  ConfigurationChangeNotification_configuration_change: ConfigurationChangeNotification_configuration_change,
  EventIdSequenceInfoNotification: EventIdSequenceInfoNotification,
  TwistLimitation: TwistLimitation,
  RobotEvent: RobotEvent,
  Base_ServiceVersion: Base_ServiceVersion,
  UserList: UserList,
  Base_SafetyIdentifier: Base_SafetyIdentifier,
  GpioPinPropertyFlags: GpioPinPropertyFlags,
  ArmStateNotification: ArmStateNotification,
  TrajectoryErrorIdentifier: TrajectoryErrorIdentifier,
  ProtectionZoneInformation: ProtectionZoneInformation,
  TrajectoryContinuityMode: TrajectoryContinuityMode,
  Pose: Pose,
  BridgePortConfig: BridgePortConfig,
  ActionHandle: ActionHandle,
  ActionList: ActionList,
  WristDigitalInputIdentifier: WristDigitalInputIdentifier,
  Snapshot: Snapshot,
  SequenceInformation: SequenceInformation,
  TrajectoryInfo: TrajectoryInfo,
  ControllerEvent: ControllerEvent,
  FactoryNotification: FactoryNotification,
  MapGroup: MapGroup,
  ArmStateInformation: ArmStateInformation,
  FullIPv4Configuration: FullIPv4Configuration,
  WifiInformationList: WifiInformationList,
  Base_RotationMatrixRow: Base_RotationMatrixRow,
  ProtectionZoneList: ProtectionZoneList,
  AdvancedSequenceHandle: AdvancedSequenceHandle,
  SequenceTaskHandle: SequenceTaskHandle,
  LedState: LedState,
  NetworkNotificationList: NetworkNotificationList,
  GpioAction: GpioAction,
  PasswordChange: PasswordChange,
  UserProfile: UserProfile,
  ControllerType: ControllerType,
  WifiSecurityType: WifiSecurityType,
  SafetyNotificationList: SafetyNotificationList,
  ChangeJointSpeeds: ChangeJointSpeeds,
  JointTorques: JointTorques,
  NavigationDirection: NavigationDirection,
  Orientation: Orientation,
  SnapshotType: SnapshotType,
  Base_GpioConfiguration: Base_GpioConfiguration,
  GpioEvent: GpioEvent,
  LimitationType: LimitationType,
  KinematicTrajectoryConstraints: KinematicTrajectoryConstraints,
  ActionNotificationList: ActionNotificationList,
  OperatingMode: OperatingMode,
  TransformationMatrix: TransformationMatrix,
  MapGroupHandle: MapGroupHandle,
  PreComputedJointTrajectoryElement: PreComputedJointTrajectoryElement,
  GpioConfigurationList: GpioConfigurationList,
  ControllerInputType: ControllerInputType,
  ControllerBehavior: ControllerBehavior,
  Xbox360AnalogInputIdentifier: Xbox360AnalogInputIdentifier,
  ChangeTwist: ChangeTwist,
  AppendActionInformation: AppendActionInformation,
  WaypointList: WaypointList,
  ConstrainedPose: ConstrainedPose,
  ServoingModeNotification: ServoingModeNotification,
  IKData: IKData,
  WifiInformation: WifiInformation,
  Base_CapSenseConfig: Base_CapSenseConfig,
  GpioCommand: GpioCommand,
  SequenceTasksPair: SequenceTasksPair,
  Finger: Finger,
  JointNavigationDirection: JointNavigationDirection,
  FirmwareComponentVersion: FirmwareComponentVersion,
  Base_Stop: Base_Stop,
  AdmittanceMode: AdmittanceMode,
  ConfigurationNotificationEvent: ConfigurationNotificationEvent,
  ConstrainedJointAngle: ConstrainedJointAngle,
  TrajectoryErrorReport: TrajectoryErrorReport,
  ActuatorInformation: ActuatorInformation,
  AngularWaypoint: AngularWaypoint,
  ControllerHandle: ControllerHandle,
  GripperCommand: GripperCommand,
  SequenceTasks: SequenceTasks,
  ConstrainedPosition: ConstrainedPosition,
  IPv4Information: IPv4Information,
  ControllerConfigurationMode: ControllerConfigurationMode,
  ActionExecutionState: ActionExecutionState,
  RequestedActionType: RequestedActionType,
  ConstrainedOrientation: ConstrainedOrientation,
  Faults: Faults,
  JointAngles: JointAngles,
  NetworkType: NetworkType,
  Query: Query,
  ConfigurationChangeNotification: ConfigurationChangeNotification,
  Admittance: Admittance,
  ProtectionZoneNotificationList: ProtectionZoneNotificationList,
  MappingHandle: MappingHandle,
  Ssid: Ssid,
  SequenceTasksConfiguration: SequenceTasksConfiguration,
  ControllerNotification_state: ControllerNotification_state,
  JointSpeed: JointSpeed,
  OperatingModeInformation: OperatingModeInformation,
  Base_CapSenseMode: Base_CapSenseMode,
  EmergencyStop: EmergencyStop,
  UserEvent: UserEvent,
  WaypointValidationReport: WaypointValidationReport,
  PreComputedJointTrajectory: PreComputedJointTrajectory,
  BridgeList: BridgeList,
  ControllerElementState: ControllerElementState,
  Point: Point,
  SequenceTasksRange: SequenceTasksRange,
  FullUserProfile: FullUserProfile,
  ConfigurationChangeNotificationList: ConfigurationChangeNotificationList,
  TrajectoryErrorElement: TrajectoryErrorElement,
  ServoingModeNotificationList: ServoingModeNotificationList,
  CartesianLimitationList: CartesianLimitationList,
  MappingInfoNotification: MappingInfoNotification,
  NetworkNotification: NetworkNotification,
  Waypoint_type_of_waypoint: Waypoint_type_of_waypoint,
  ControllerConfiguration: ControllerConfiguration,
  SwitchControlMapping: SwitchControlMapping,
  OperatingModeNotificationList: OperatingModeNotificationList,
  GpioPinConfiguration: GpioPinConfiguration,
  ControllerEventType: ControllerEventType,
  ZoneShape: ZoneShape,
  BaseCyclic_ServiceVersion: BaseCyclic_ServiceVersion,
  BaseCyclic_CustomData: BaseCyclic_CustomData,
  ActuatorCommand: ActuatorCommand,
  ActuatorFeedback: ActuatorFeedback,
  BaseCyclic_Command: BaseCyclic_Command,
  BaseCyclic_Feedback: BaseCyclic_Feedback,
  ActuatorCustomData: ActuatorCustomData,
  BaseFeedback: BaseFeedback,
  UARTParity: UARTParity,
  Empty: Empty,
  UARTDeviceIdentification: UARTDeviceIdentification,
  Unit: Unit,
  CountryCode: CountryCode,
  DeviceTypes: DeviceTypes,
  UARTStopBits: UARTStopBits,
  ArmState: ArmState,
  Connection: Connection,
  UARTSpeed: UARTSpeed,
  UARTWordLength: UARTWordLength,
  NotificationOptions: NotificationOptions,
  UARTConfiguration: UARTConfiguration,
  DeviceHandle: DeviceHandle,
  NotificationType: NotificationType,
  Permission: Permission,
  SafetyNotification: SafetyNotification,
  SafetyHandle: SafetyHandle,
  CountryCodeIdentifier: CountryCodeIdentifier,
  Timestamp: Timestamp,
  CartesianReferenceFrame: CartesianReferenceFrame,
  NotificationHandle: NotificationHandle,
  UserProfileHandle: UserProfileHandle,
  SafetyStatusValue: SafetyStatusValue,
  JointAccelerationSoftLimits: JointAccelerationSoftLimits,
  DesiredSpeeds: DesiredSpeeds,
  GravityVector: GravityVector,
  ControlConfigurationEvent: ControlConfigurationEvent,
  TwistLinearSoftLimit: TwistLinearSoftLimit,
  ControlConfig_JointSpeeds: ControlConfig_JointSpeeds,
  KinematicLimits: KinematicLimits,
  LinearTwist: LinearTwist,
  CartesianTransform: CartesianTransform,
  JointSpeedSoftLimits: JointSpeedSoftLimits,
  ControlConfig_ServiceVersion: ControlConfig_ServiceVersion,
  TwistAngularSoftLimit: TwistAngularSoftLimit,
  ControlConfig_ControlMode: ControlConfig_ControlMode,
  CartesianReferenceFrameInfo: CartesianReferenceFrameInfo,
  ToolConfiguration: ToolConfiguration,
  AngularTwist: AngularTwist,
  ControlConfig_ControlModeInformation: ControlConfig_ControlModeInformation,
  KinematicLimitsList: KinematicLimitsList,
  ControlConfig_Position: ControlConfig_Position,
  PayloadInformation: PayloadInformation,
  ControlConfigurationNotification: ControlConfigurationNotification,
  ControlConfig_ControlModeNotification: ControlConfig_ControlModeNotification,
  CalibrationItem: CalibrationItem,
  SafetyConfigurationList: SafetyConfigurationList,
  BootloaderVersion: BootloaderVersion,
  SafetyStatus: SafetyStatus,
  SerialNumber: SerialNumber,
  Calibration: Calibration,
  DeviceConfig_ServiceVersion: DeviceConfig_ServiceVersion,
  CalibrationParameter: CalibrationParameter,
  MACAddress: MACAddress,
  PartNumber: PartNumber,
  PartNumberRevision: PartNumberRevision,
  SafetyEnable: SafetyEnable,
  FirmwareVersion: FirmwareVersion,
  DeviceType: DeviceType,
  CalibrationParameter_value: CalibrationParameter_value,
  CalibrationElement: CalibrationElement,
  DeviceConfig_CapSenseConfig: DeviceConfig_CapSenseConfig,
  SafetyConfiguration: SafetyConfiguration,
  DeviceConfig_CapSenseMode: DeviceConfig_CapSenseMode,
  SafetyThreshold: SafetyThreshold,
  DeviceConfig_SafetyLimitType: DeviceConfig_SafetyLimitType,
  CalibrationStatus: CalibrationStatus,
  CalibrationResult: CalibrationResult,
  IPv4Settings: IPv4Settings,
  RunModes: RunModes,
  SafetyInformationList: SafetyInformationList,
  RunMode: RunMode,
  PowerOnSelfTestResult: PowerOnSelfTestResult,
  SafetyInformation: SafetyInformation,
  CapSenseRegister: CapSenseRegister,
  RebootRqst: RebootRqst,
  ModelNumber: ModelNumber,
  DeviceHandles: DeviceHandles,
  DeviceManager_ServiceVersion: DeviceManager_ServiceVersion,
  RobotiqGripperStatusFlags: RobotiqGripperStatusFlags,
  GripperConfig_SafetyIdentifier: GripperConfig_SafetyIdentifier,
  GripperCyclic_MessageId: GripperCyclic_MessageId,
  CustomDataUnit: CustomDataUnit,
  GripperCyclic_ServiceVersion: GripperCyclic_ServiceVersion,
  GripperCyclic_Command: GripperCyclic_Command,
  MotorCommand: MotorCommand,
  GripperCyclic_CustomData: GripperCyclic_CustomData,
  GripperCyclic_Feedback: GripperCyclic_Feedback,
  MotorFeedback: MotorFeedback,
  GPIOState: GPIOState,
  GPIOIdentification: GPIOIdentification,
  I2CWriteRegisterParameter: I2CWriteRegisterParameter,
  UARTPortId: UARTPortId,
  I2CReadRegisterParameter: I2CReadRegisterParameter,
  EthernetDeviceIdentification: EthernetDeviceIdentification,
  InterconnectConfig_GPIOConfiguration: InterconnectConfig_GPIOConfiguration,
  I2CData: I2CData,
  InterconnectConfig_SafetyIdentifier: InterconnectConfig_SafetyIdentifier,
  I2CRegisterAddressSize: I2CRegisterAddressSize,
  EthernetDuplex: EthernetDuplex,
  I2CDeviceIdentification: I2CDeviceIdentification,
  I2CWriteParameter: I2CWriteParameter,
  EthernetConfiguration: EthernetConfiguration,
  I2CMode: I2CMode,
  I2CConfiguration: I2CConfiguration,
  EthernetSpeed: EthernetSpeed,
  EthernetDevice: EthernetDevice,
  GPIOMode: GPIOMode,
  GPIOIdentifier: GPIOIdentifier,
  InterconnectConfig_ServiceVersion: InterconnectConfig_ServiceVersion,
  GPIOPull: GPIOPull,
  I2CDeviceAddressing: I2CDeviceAddressing,
  GPIOValue: GPIOValue,
  I2CDevice: I2CDevice,
  I2CReadParameter: I2CReadParameter,
  InterconnectCyclic_MessageId: InterconnectCyclic_MessageId,
  InterconnectCyclic_ServiceVersion: InterconnectCyclic_ServiceVersion,
  InterconnectCyclic_Feedback_tool_feedback: InterconnectCyclic_Feedback_tool_feedback,
  InterconnectCyclic_Feedback: InterconnectCyclic_Feedback,
  InterconnectCyclic_Command: InterconnectCyclic_Command,
  InterconnectCyclic_Command_tool_command: InterconnectCyclic_Command_tool_command,
  InterconnectCyclic_CustomData: InterconnectCyclic_CustomData,
  InterconnectCyclic_CustomData_tool_customData: InterconnectCyclic_CustomData_tool_customData,
  ModelId: ModelId,
  WristType: WristType,
  BaseType: BaseType,
  CompleteProductConfiguration: CompleteProductConfiguration,
  ProductConfigurationEndEffectorType: ProductConfigurationEndEffectorType,
  InterfaceModuleType: InterfaceModuleType,
  EndEffectorType: EndEffectorType,
  ArmLaterality: ArmLaterality,
  VisionModuleType: VisionModuleType,
  FocusAction: FocusAction,
  ExtrinsicParameters: ExtrinsicParameters,
  OptionIdentifier: OptionIdentifier,
  BitRate: BitRate,
  SensorFocusAction: SensorFocusAction,
  IntrinsicProfileIdentifier: IntrinsicProfileIdentifier,
  VisionConfig_ServiceVersion: VisionConfig_ServiceVersion,
  DistortionCoefficients: DistortionCoefficients,
  OptionValue: OptionValue,
  Option: Option,
  VisionEvent: VisionEvent,
  Resolution: Resolution,
  SensorIdentifier: SensorIdentifier,
  IntrinsicParameters: IntrinsicParameters,
  FocusPoint: FocusPoint,
  VisionConfig_RotationMatrix: VisionConfig_RotationMatrix,
  SensorFocusAction_action_parameters: SensorFocusAction_action_parameters,
  VisionConfig_RotationMatrixRow: VisionConfig_RotationMatrixRow,
  ManualFocus: ManualFocus,
  Sensor: Sensor,
  FrameRate: FrameRate,
  TranslationVector: TranslationVector,
  OptionInformation: OptionInformation,
  VisionNotification: VisionNotification,
  SensorSettings: SensorSettings,
  FollowCartesianTrajectoryAction: FollowCartesianTrajectoryAction,
  FollowCartesianTrajectoryActionGoal: FollowCartesianTrajectoryActionGoal,
  FollowCartesianTrajectoryActionFeedback: FollowCartesianTrajectoryActionFeedback,
  FollowCartesianTrajectoryGoal: FollowCartesianTrajectoryGoal,
  FollowCartesianTrajectoryResult: FollowCartesianTrajectoryResult,
  FollowCartesianTrajectoryFeedback: FollowCartesianTrajectoryFeedback,
  FollowCartesianTrajectoryActionResult: FollowCartesianTrajectoryActionResult,
};

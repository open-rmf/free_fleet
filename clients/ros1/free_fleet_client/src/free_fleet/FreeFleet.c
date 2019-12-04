/****************************************************************

  Generated by Eclipse Cyclone DDS IDL to C Translator
  File name: FreeFleet.c
  Source: FreeFleet.idl
  Cyclone DDS: V0.5.0

*****************************************************************/
#include "FreeFleet.h"


static const uint32_t FreeFleetData_RobotMode_ops [] =
{
  DDS_OP_ADR | DDS_OP_TYPE_4BY, offsetof (FreeFleetData_RobotMode, mode),
  DDS_OP_RTS
};

const dds_topic_descriptor_t FreeFleetData_RobotMode_desc =
{
  sizeof (FreeFleetData_RobotMode),
  4u,
  0u,
  0u,
  "FreeFleetData::RobotMode",
  NULL,
  2,
  FreeFleetData_RobotMode_ops,
  "<MetaData version=\"1.0.0\"><Module name=\"FreeFleetData\"><Struct name=\"RobotMode\"><Member name=\"mode\"><ULong/></Member></Struct></Module></MetaData>"
};


static const uint32_t FreeFleetData_Location_ops [] =
{
  DDS_OP_ADR | DDS_OP_TYPE_4BY, offsetof (FreeFleetData_Location, sec),
  DDS_OP_ADR | DDS_OP_TYPE_4BY, offsetof (FreeFleetData_Location, nanosec),
  DDS_OP_ADR | DDS_OP_TYPE_4BY, offsetof (FreeFleetData_Location, x),
  DDS_OP_ADR | DDS_OP_TYPE_4BY, offsetof (FreeFleetData_Location, y),
  DDS_OP_ADR | DDS_OP_TYPE_4BY, offsetof (FreeFleetData_Location, yaw),
  DDS_OP_ADR | DDS_OP_TYPE_STR, offsetof (FreeFleetData_Location, level_name),
  DDS_OP_RTS
};

const dds_topic_descriptor_t FreeFleetData_Location_desc =
{
  sizeof (FreeFleetData_Location),
  sizeof (char *),
  DDS_TOPIC_NO_OPTIMIZE,
  0u,
  "FreeFleetData::Location",
  NULL,
  7,
  FreeFleetData_Location_ops,
  "<MetaData version=\"1.0.0\"><Module name=\"FreeFleetData\"><Struct name=\"Location\"><Member name=\"sec\"><Long/></Member><Member name=\"nanosec\"><ULong/></Member><Member name=\"x\"><Float/></Member><Member name=\"y\"><Float/></Member><Member name=\"yaw\"><Float/></Member><Member name=\"level_name\"><String/></Member></Struct></Module></MetaData>"
};


static const uint32_t FreeFleetData_RobotState_ops [] =
{
  DDS_OP_ADR | DDS_OP_TYPE_STR, offsetof (FreeFleetData_RobotState, name),
  DDS_OP_ADR | DDS_OP_TYPE_STR, offsetof (FreeFleetData_RobotState, model),
  DDS_OP_ADR | DDS_OP_TYPE_STR, offsetof (FreeFleetData_RobotState, task_id),
  DDS_OP_ADR | DDS_OP_TYPE_4BY, offsetof (FreeFleetData_RobotState, mode.mode),
  DDS_OP_ADR | DDS_OP_TYPE_4BY, offsetof (FreeFleetData_RobotState, battery_percent),
  DDS_OP_ADR | DDS_OP_TYPE_4BY, offsetof (FreeFleetData_RobotState, location.sec),
  DDS_OP_ADR | DDS_OP_TYPE_4BY, offsetof (FreeFleetData_RobotState, location.nanosec),
  DDS_OP_ADR | DDS_OP_TYPE_4BY, offsetof (FreeFleetData_RobotState, location.x),
  DDS_OP_ADR | DDS_OP_TYPE_4BY, offsetof (FreeFleetData_RobotState, location.y),
  DDS_OP_ADR | DDS_OP_TYPE_4BY, offsetof (FreeFleetData_RobotState, location.yaw),
  DDS_OP_ADR | DDS_OP_TYPE_STR, offsetof (FreeFleetData_RobotState, location.level_name),
  DDS_OP_ADR | DDS_OP_TYPE_SEQ | DDS_OP_SUBTYPE_STU, offsetof (FreeFleetData_RobotState, path),
  sizeof (FreeFleetData_Location), (17u << 16u) + 4u,
  DDS_OP_ADR | DDS_OP_TYPE_4BY, offsetof (FreeFleetData_Location, sec),
  DDS_OP_ADR | DDS_OP_TYPE_4BY, offsetof (FreeFleetData_Location, nanosec),
  DDS_OP_ADR | DDS_OP_TYPE_4BY, offsetof (FreeFleetData_Location, x),
  DDS_OP_ADR | DDS_OP_TYPE_4BY, offsetof (FreeFleetData_Location, y),
  DDS_OP_ADR | DDS_OP_TYPE_4BY, offsetof (FreeFleetData_Location, yaw),
  DDS_OP_ADR | DDS_OP_TYPE_STR, offsetof (FreeFleetData_Location, level_name),
  DDS_OP_RTS,
  DDS_OP_RTS
};

const dds_topic_descriptor_t FreeFleetData_RobotState_desc =
{
  sizeof (FreeFleetData_RobotState),
  sizeof (char *),
  DDS_TOPIC_NO_OPTIMIZE,
  0u,
  "FreeFleetData::RobotState",
  NULL,
  21,
  FreeFleetData_RobotState_ops,
  "<MetaData version=\"1.0.0\"><Module name=\"FreeFleetData\"><Struct name=\"RobotMode\"><Member name=\"mode\"><ULong/></Member></Struct><Struct name=\"Location\"><Member name=\"sec\"><Long/></Member><Member name=\"nanosec\"><ULong/></Member><Member name=\"x\"><Float/></Member><Member name=\"y\"><Float/></Member><Member name=\"yaw\"><Float/></Member><Member name=\"level_name\"><String/></Member></Struct><Struct name=\"RobotState\"><Member name=\"name\"><String/></Member><Member name=\"model\"><String/></Member><Member name=\"task_id\"><String/></Member><Member name=\"mode\"><Type name=\"RobotMode\"/></Member><Member name=\"battery_percent\"><Float/></Member><Member name=\"location\"><Type name=\"Location\"/></Member><Member name=\"path\"><Sequence><Type name=\"Location\"/></Sequence></Member></Struct></Module></MetaData>"
};


static const uint32_t FreeFleetData_ModeRequest_ops [] =
{
  DDS_OP_ADR | DDS_OP_TYPE_4BY, offsetof (FreeFleetData_ModeRequest, location.sec),
  DDS_OP_ADR | DDS_OP_TYPE_4BY, offsetof (FreeFleetData_ModeRequest, location.nanosec),
  DDS_OP_ADR | DDS_OP_TYPE_4BY, offsetof (FreeFleetData_ModeRequest, location.x),
  DDS_OP_ADR | DDS_OP_TYPE_4BY, offsetof (FreeFleetData_ModeRequest, location.y),
  DDS_OP_ADR | DDS_OP_TYPE_4BY, offsetof (FreeFleetData_ModeRequest, location.yaw),
  DDS_OP_ADR | DDS_OP_TYPE_STR, offsetof (FreeFleetData_ModeRequest, location.level_name),
  DDS_OP_ADR | DDS_OP_TYPE_STR, offsetof (FreeFleetData_ModeRequest, task_id),
  DDS_OP_RTS
};

const dds_topic_descriptor_t FreeFleetData_ModeRequest_desc =
{
  sizeof (FreeFleetData_ModeRequest),
  sizeof (char *),
  DDS_TOPIC_NO_OPTIMIZE,
  0u,
  "FreeFleetData::ModeRequest",
  NULL,
  8,
  FreeFleetData_ModeRequest_ops,
  "<MetaData version=\"1.0.0\"><Module name=\"FreeFleetData\"><Struct name=\"Location\"><Member name=\"sec\"><Long/></Member><Member name=\"nanosec\"><ULong/></Member><Member name=\"x\"><Float/></Member><Member name=\"y\"><Float/></Member><Member name=\"yaw\"><Float/></Member><Member name=\"level_name\"><String/></Member></Struct><Struct name=\"ModeRequest\"><Member name=\"location\"><Type name=\"Location\"/></Member><Member name=\"task_id\"><String/></Member></Struct></Module></MetaData>"
};


static const uint32_t FreeFleetData_PathRequest_ops [] =
{
  DDS_OP_ADR | DDS_OP_TYPE_SEQ | DDS_OP_SUBTYPE_STU, offsetof (FreeFleetData_PathRequest, path),
  sizeof (FreeFleetData_Location), (17u << 16u) + 4u,
  DDS_OP_ADR | DDS_OP_TYPE_4BY, offsetof (FreeFleetData_Location, sec),
  DDS_OP_ADR | DDS_OP_TYPE_4BY, offsetof (FreeFleetData_Location, nanosec),
  DDS_OP_ADR | DDS_OP_TYPE_4BY, offsetof (FreeFleetData_Location, x),
  DDS_OP_ADR | DDS_OP_TYPE_4BY, offsetof (FreeFleetData_Location, y),
  DDS_OP_ADR | DDS_OP_TYPE_4BY, offsetof (FreeFleetData_Location, yaw),
  DDS_OP_ADR | DDS_OP_TYPE_STR, offsetof (FreeFleetData_Location, level_name),
  DDS_OP_RTS,
  DDS_OP_ADR | DDS_OP_TYPE_STR, offsetof (FreeFleetData_PathRequest, task_id),
  DDS_OP_RTS
};

const dds_topic_descriptor_t FreeFleetData_PathRequest_desc =
{
  sizeof (FreeFleetData_PathRequest),
  sizeof (char *),
  DDS_TOPIC_NO_OPTIMIZE,
  0u,
  "FreeFleetData::PathRequest",
  NULL,
  11,
  FreeFleetData_PathRequest_ops,
  "<MetaData version=\"1.0.0\"><Module name=\"FreeFleetData\"><Struct name=\"Location\"><Member name=\"sec\"><Long/></Member><Member name=\"nanosec\"><ULong/></Member><Member name=\"x\"><Float/></Member><Member name=\"y\"><Float/></Member><Member name=\"yaw\"><Float/></Member><Member name=\"level_name\"><String/></Member></Struct><Struct name=\"PathRequest\"><Member name=\"path\"><Sequence><Type name=\"Location\"/></Sequence></Member><Member name=\"task_id\"><String/></Member></Struct></Module></MetaData>"
};


static const uint32_t FreeFleetData_DestinationRequest_ops [] =
{
  DDS_OP_ADR | DDS_OP_TYPE_4BY, offsetof (FreeFleetData_DestinationRequest, location.sec),
  DDS_OP_ADR | DDS_OP_TYPE_4BY, offsetof (FreeFleetData_DestinationRequest, location.nanosec),
  DDS_OP_ADR | DDS_OP_TYPE_4BY, offsetof (FreeFleetData_DestinationRequest, location.x),
  DDS_OP_ADR | DDS_OP_TYPE_4BY, offsetof (FreeFleetData_DestinationRequest, location.y),
  DDS_OP_ADR | DDS_OP_TYPE_4BY, offsetof (FreeFleetData_DestinationRequest, location.yaw),
  DDS_OP_ADR | DDS_OP_TYPE_STR, offsetof (FreeFleetData_DestinationRequest, location.level_name),
  DDS_OP_ADR | DDS_OP_TYPE_STR, offsetof (FreeFleetData_DestinationRequest, task_id),
  DDS_OP_RTS
};

const dds_topic_descriptor_t FreeFleetData_DestinationRequest_desc =
{
  sizeof (FreeFleetData_DestinationRequest),
  sizeof (char *),
  DDS_TOPIC_NO_OPTIMIZE,
  0u,
  "FreeFleetData::DestinationRequest",
  NULL,
  8,
  FreeFleetData_DestinationRequest_ops,
  "<MetaData version=\"1.0.0\"><Module name=\"FreeFleetData\"><Struct name=\"Location\"><Member name=\"sec\"><Long/></Member><Member name=\"nanosec\"><ULong/></Member><Member name=\"x\"><Float/></Member><Member name=\"y\"><Float/></Member><Member name=\"yaw\"><Float/></Member><Member name=\"level_name\"><String/></Member></Struct><Struct name=\"DestinationRequest\"><Member name=\"location\"><Type name=\"Location\"/></Member><Member name=\"task_id\"><String/></Member></Struct></Module></MetaData>"
};

from typing import Any, Dict, List, Optional

from pydantic import BaseModel, Field


class ConnectRequest(BaseModel):
    host: str = Field(..., min_length=1)
    port: int = Field(23, ge=1, le=65535)


class DisconnectResponse(BaseModel):
    disconnected: bool


class CommandRequest(BaseModel):
    command_id: str = Field(..., min_length=1)
    args: Dict[str, Any] = Field(default_factory=dict)
    confirmed: bool = False


class RawCommandRequest(BaseModel):
    command: str = Field(..., min_length=1)


class CommandArgDefinition(BaseModel):
    name: str
    type: str
    label: str
    required: bool = True
    min: Optional[float] = None
    max: Optional[float] = None
    step: Optional[float] = None
    default: Optional[Any] = None


class CommandDefinition(BaseModel):
    id: str
    label: str
    group: str
    template: Optional[str] = None
    args: List[CommandArgDefinition] = Field(default_factory=list)
    requires_confirmation: bool = False
    confirm_message: Optional[str] = None
    poll_after: List[str] = Field(default_factory=list)


class CommandExecutionResult(BaseModel):
    command_id: str
    rendered_command: str
    queued: bool


class WebSocketEvent(BaseModel):
    type: str
    data: Dict[str, Any]

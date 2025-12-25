"""create chat sessions and messages tables

Revision ID: f20f09926f5f
Revises: 019c33fc5007
Create Date: 2025-12-20 04:49:30.273040

"""
from typing import Sequence, Union

from alembic import op
import sqlalchemy as sa


# revision identifiers, used by Alembic.
revision: str = 'f20f09926f5f'
down_revision: Union[str, Sequence[str], None] = '019c33fc5007'
branch_labels: Union[str, Sequence[str], None] = None
depends_on: Union[str, Sequence[str], None] = None


def upgrade() -> None:
    """Upgrade schema."""
    pass


def downgrade() -> None:
    """Downgrade schema."""
    pass

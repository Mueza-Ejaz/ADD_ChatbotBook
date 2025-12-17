"""create chat sessions and messages tables

Revision ID: 019c33fc5007
Revises: 6472c39ecba8
Create Date: 2025-12-17 21:39:43.004552

"""
from typing import Sequence, Union

from alembic import op
import sqlalchemy as sa


# revision identifiers, used by Alembic.
revision: str = '019c33fc5007'
down_revision: Union[str, Sequence[str], None] = '6472c39ecba8'
branch_labels: Union[str, Sequence[str], None] = None
depends_on: Union[str, Sequence[str], None] = None


def upgrade() -> None:
    """Upgrade schema."""
    pass


def downgrade() -> None:
    """Downgrade schema."""
    pass

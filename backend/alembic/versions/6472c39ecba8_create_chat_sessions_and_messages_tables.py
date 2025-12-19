"""create chat sessions and messages tables

Revision ID: 6472c39ecba8
Revises: ffdad0695bdd
Create Date: 2025-12-17 18:13:48.065993

"""
from typing import Sequence, Union

from alembic import op
import sqlalchemy as sa


# revision identifiers, used by Alembic.
revision: str = '6472c39ecba8'
down_revision: Union[str, Sequence[str], None] = 'ffdad0695bdd'
branch_labels: Union[str, Sequence[str], None] = None
depends_on: Union[str, Sequence[str], None] = None


def upgrade() -> None:
    """Upgrade schema."""
    pass


def downgrade() -> None:
    """Downgrade schema."""
    pass

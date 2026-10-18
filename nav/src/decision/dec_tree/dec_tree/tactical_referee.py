"""Compatibility names used by the existing tactical conditions.

The current Referee message carries catch/defend/bumpy fields. The old combined
pilot_cmd field has no wire equivalent and is supplied explicitly by the operator
on /tactical/pilot_cmd (0=none, 1=mid, 2=enemy base).
"""
class TacticalReferee:
    _aliases = {
        'enemy_hero_pos': 'catch_hero',
        'enemy_engineer_pos': 'catch_engineer',
        'fortress_enemy': 'defend_fortress',
        'bumpy_road': 'bumpy_exist_enemy',
    }

    def __init__(self, message, pilot_cmd=0):
        self.message = message
        self.pilot_cmd = pilot_cmd

    def __getattr__(self, name):
        return getattr(self.message, self._aliases.get(name, name))

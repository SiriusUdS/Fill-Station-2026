# Integration tests (stm-2026-common)

Reserved for tests that exercise several of this submodule's units together
(as opposed to `../unit/`, which tests one unit in isolation).

Empty for now: the submodule's units are mostly pure transforms (parsers,
persistent state) that are covered as units, and the cross-module command
pipeline is assembled in the parent repo (its handlers live there), so the first
integration test — an Ethernet frame driving a state change — lives in the
parent's `tests/integration/`. Add submodule-only integration tests here as the
common code grows ones worth assembling.

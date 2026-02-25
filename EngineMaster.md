# ENGINE MASTER — Fichier de suivi projet
> Dernière mise à jour : 2026-02-24
> SDL3 | C++20 | CMake | Git Submodules | vcpkg

---

## 🤖 CONTEXTE POUR CLAUDE CODE

### Qui est l'utilisateur
Eliott, étudiant GTECH1 à Gaming Campus Lyon. Il apprend le développement de moteur de jeu en C++.

### Règles pédagogiques IMPORTANTES
- **Ne jamais générer du code complet** sauf si Eliott dit explicitement "code-moi ça", "écris le code", "génère la fonction"
- À la place : guider par étapes, donner la structure avec commentaires, poser des questions
- Pointer les erreurs avec précision (numéro de ligne), expliquer POURQUOI, donner des indices pas la correction
- Forcer un codage générique et durable
- Poser des questions de clarification avant de répondre

### État d'avancement au moment du transfert
- Architecture globale : ✅ définie
- Setup VS Code + CMake + vcpkg : ✅ fonctionnel
- Repo `eliott-ecs` : ✅ créé localement (`C:\Dev\eliott-ecs`)
- **Tâche en cours** : eliott-physics 🔴 À créer (eliott-math ✅ terminé)

### Ce qu'Eliott a déjà compris (bases théoriques validées)
- ECS : Entity = juste un uint32_t (ID), Component = data only, System = logique
- Stockage par tableau par type de component (approche cache-friendly)
- `unordered_map<EntityID, size_t>` pour l'index
- Recyclage des IDs via `std::queue` + `std::unordered_set` pour les vivants
- La queue est vide au départ, on prend `_nextID` si vide, sinon on pop

---

## 🗺️ Vue d'ensemble architecture

```
[game-project]          ← Repo utilisateur final (jeu)
      │
      └── [engine]      ← Repo moteur principal
              ├── [ecs]          ← submodule
              ├── [tmx-parser]   ← submodule
              ├── [renderer]     ← submodule
              ├── [audio]        ← submodule
              ├── [input]        ← submodule
              └── [physics]      ← submodule
```

---

## 📦 Repos & responsabilités

### 1. `eliott-ecs`
- **Rôle** : Système ECS générique, réutilisable par n'importe quel projet
- **Dépendances externes** : doctest (tests uniquement)
- **Expose** : `World`, `Entity`, `ComponentManager`, `SystemManager`
- **Standard** : C++20, lib statique
- **Status** : ✅ Terminé

### 2. `eliott-tmx-parser`
- **Rôle** : Parser de fichiers `.tmx` (Tiled Map Editor)
- **Dépendances externes** : tinyxml2
- **Expose** : `TmxMap`, `TmxLayer`, `TmxTileset`, `TmxObject`, `TmxObjectGroup`
- **Status** : ✅ Terminé

### 3. `eliott-renderer`
- **Rôle** : Abstraction du rendu 2D via SDL3
- **Dépendances externes** : SDL3, SDL3_image
- **Expose** : `Renderer`, `Texture`, `Camera`, `SpriteBatch`, `SpriteEntry`, `Sprite`
- **Dépend de** : `eliott-ecs`, `eliott-math`
- **Note** : `Sprite` (composant ECS visuel, wraps Texture) vit ici car il dépend de SDL3.
- **Status** : ✅ Terminé (Sprite à ajouter lors de eliott-engine)

### 4. `eliott-audio`
- **Rôle** : Gestion des sons et musiques
- **Dépendances externes** : miniaudio (single-header, vcpkg `x64-mingw-static`)
- **Expose** : `AudioManager`, `AudioMap<Event, Group>`
- **Status** : ✅ Terminé (sans tests)
- **Note** : Backend miniaudio (pas SDL3_mixer, non dispo sur vcpkg).
  `AudioManager` singleton wraps `ma_engine`, stocke `vector<ma_sound>` (index = SoundHandle).
  `AudioMap<T, G>` template haut niveau, pattern identique à `ActionMap<T>` de l'input.
  Support groupes (`ma_sound_group`), volume par son / groupe / global, fade in/out en PCM frames.

### 5. `eliott-input`
- **Rôle** : Gestion des entrées clavier/souris/gamepad
- **Dépendances externes** : SDL3
- **Expose** : `InputManager`, `ActionMap`, `KeyBinding`
- **Status** : ✅ Terminé

### 6. `eliott-physics`
- **Rôle** : Physique rigide 2D + détection via Quadtree
- **Dépendances externes** : aucune
- **Expose** : `PhysicsWorld`, `RigidBody`, `Collider`, `Quadtree`
- **Dépend de** : `eliott-ecs`, `eliott-math`
- **Composants ECS** : `RigidBody` (velocity, mass, isStatic), `Collider` (std::variant<AABB, Circle>)
- **Note** : Position via `Transform` (eliott-math). `PhysicsWorld` interroge le `ee::ecs::World` pour trouver les entités avec RigidBody + Collider + Transform.
- **Status** : 🔴 À créer (après eliott-math)

### 7. `eliott-math`
- **Rôle** : Types mathématiques et composants de base partagés entre tous les modules
- **Dépendances externes** : aucune
- **Expose** : `Vector2<T>`, `Rect<T>`, `Transform`
- **Dépend de** : rien
- **Note** : `Transform` vit ici car il est partagé par renderer ET physics sans créer de dépendance croisée. `Sprite` vit dans `eliott-renderer` (dépend de SDL3/Texture).
- **Status** : ✅ Terminé

### 8. `eliott-engine`
- **Rôle** : Moteur principal, assemble tous les modules
- **Dépendances** : tous les submodules + SDL3
- **Expose** : `Engine`, `Scene`, `SceneManager`, `GameLoop`
- **Status** : 🔴 À créer

---

## 🔗 Matrice de dépendances inter-modules

| Module       | ecs | tmx | math | renderer | audio | input | physics |
|--------------|:---:|:---:|:----:|:--------:|:-----:|:-----:|:-------:|
| ecs          |  —  |     |      |          |       |       |         |
| tmx-parser   |     |  —  |      |          |       |       |         |
| math         |     |     |  —   |          |       |       |         |
| renderer     |  ✓  |  ✓  |  ✓   |    —     |       |       |         |
| audio        |     |     |      |          |   —   |       |         |
| input        |     |     |      |          |       |   —   |         |
| physics      |  ✓  |     |  ✓   |          |       |       |    —    |
| **engine**   |  ✓  |  ✓  |  ✓   |    ✓     |   ✓   |   ✓   |    ✓    |

---

## 🗂️ Structure type de chaque repo

```
eliott-<module>/
├── CMakeLists.txt
├── README.md
├── include/
│   └── <module>/
│       └── *.hpp          ← API publique
├── src/
│   └── *.cpp
└── tests/
    └── test_*.cpp
```

---

## ✅ Ordre de développement

```
Phase 1 — Fondations
  ✅ eliott-ecs
    ✅ EntityManager
    ✅ ComponentArray<T> + IComponentArray
    ✅ ComponentManager
    ✅ SystemManager
    ✅ World (façade)
    ✅ Tests doctest (tous passent)
  ✅ eliott-tmx-parser
    ✅ Headers (TmxMap, TmxLayer, TmxTileset, TmxObject, TmxObjectGroup)
    ✅ CMakeLists.txt (tinyxml2, STATIC)
    ✅ TmxParser::load
    ✅ TmxParser::loadTileSet (tsx externe + inline)
    ✅ TmxParser::loadLayer (CSV parsing)
    ✅ TmxObjectGroup struct + refacto TmxMap::m_objectGroup
    ✅ TmxParser::loadObjectGroup + loadObject
    ✅ Tests doctest (tous passent)

Phase 2 — Modules SDL3
  ✅ eliott-input
    ✅ InputManager (clavier, souris, gamepad, singleton)
    ✅ KeyBinding.hpp (TriggerState, Input variant, KeyBinding)
    ✅ ActionMap<T> (bind, isActive, OR/ET)
    ✅ Tests doctest (tous passent)
  ✅ eliott-audio

Phase 3 — Modules ECS-dépendants
  ✅ eliott-renderer
  ✅ eliott-math (Vector2, Rect, Transform)
  🔴 eliott-physics (Quadtree + RigidBody + Collider)

Phase 4 — Assemblage
  🔴 eliott-engine

Phase 5 — Validation
  🔴 Mini-jeu de test avec map Tiled
```

---

## 🔧 Conventions de code

- Namespace : `ee::ecs::`, `ee::physics::`, `ee::renderer::`, etc.
- Membres privés préfixés `m_` : `m_availableIDs`, `m_nextID`
- Arguments des fonctions préfixés `_` : `_path`, `_id` 
- Pas de `using namespace` dans les headers
- RAII strict, pas de `new`/`delete` raw
- `std::optional` pour les retours faillibles
- Interfaces → classe abstraite pure ou concept C++20
- Code propre et lisible : éviter la redondance (ex: appeler explicitement un constructeur par défaut que le compilateur appellerait de toute façon). Préférer `= default` pour les constructeurs/destructeurs générés par le compilateur.

---

## 📋 Décisions actées

| Sujet | Décision |
|-------|----------|
| SDL | SDL3 |
| Build | CMake |
| Dépendances | vcpkg |
| Tests | doctest |
| Lib | Statique (.lib) |
| ECS RuneBorn | Refactorisé from scratch |
| Stockage components | unordered_map<EntityID, T> par type |
| Recyclage IDs | std::queue + std::unordered_set |
| TMX ObjectGroup | struct TmxObjectGroup (id, name, vector<TmxObject>) |
| TMX data encoding | CSV uniquement (base64 non supporté) |
| Backend audio | miniaudio (single-header) |

## 📋 Décisions en suspens

| Sujet | Question |
|-------|----------|
| Encapsulation SDL3 input | `SDL_Scancode` et `SDL_GamepadButton` exposés dans l'API publique de `eliott-input`. À remplacer par `ee::input::Key` et `ee::input::GamepadButton` (enums maison mappés en interne). À faire avant ou pendant `eliott-engine`.|

---

## 📌 Notes de session

```
[2026-02-21] Session 1 — Initialisation complète du projet.
             Architecture définie, setup CMake+vcpkg fonctionnel.
             Bases théoriques ECS validées avec Eliott.
             EntityManager.hpp écrit, .cpp à implémenter par Eliott.

[2026-02-21] Session 2 — eliott-ecs complété.
             Implémenté : ComponentArray<T>, IComponentArray, ComponentManager,
             ComponentRegistry (Signature/getComponentID), System, SystemManager, World.
             Tous les tests passent (doctest).
             Concepts clés vus : type erasure, std::type_index, std::static_pointer_cast,
             inline variables (ODR), bitmask signatures, façade pattern.
             Prochaine session : eliott-tmx-parser.

[2026-02-22] Session 3 — eliott-tmx-parser démarré.
             Tour du format TMX (XML, tilesets, layers, objectgroups, GIDs).
             Headers écrits : TmxMap, TmxLayer, TmxTileset, TmxObject.
             CMakeLists.txt fonctionnel (tinyxml2 via vcpkg, lib STATIC).
             TmxParser::load, loadTileSet (tsx externe + inline), loadLayer (CSV) implémentés.
             Concepts vus : std::filesystem::path, std::optional, std::nullopt,
             if(const char* x = ...) pattern, file-static functions.

[2026-02-23] Session 4 — eliott-tmx-parser complété.
             TmxObjectGroup struct, loadObjectGroup, loadObject implémentés.
             Tests doctest écrits et passent tous.
             Problème résolu : triplet vcpkg x64-windows (MSVC) incompatible avec MinGW.
             Fix : vcpkg install tinyxml2:x64-mingw-static + CMAKE_PREFIX_PATH mis à jour.
             Concepts vus : triplets vcpkg, ABI incompatibilité MSVC/MinGW,
             file(COPY ...) CMake pour les fixtures de test.
             Prochaine session : eliott-renderer (SDL3).

[2026-02-23] Session 5 — eliott-renderer complété.
             CMakeLists.txt (SDL3, SDL3_image, eliott-ecs submodule, triplet mingw-static).
             Implémenté : Renderer (fenêtre + SDL_Renderer, RAII, throw dans constructeur),
             Texture (IMG_LoadTexture, constructeur privé + friend Renderer, copy deleted),
             Camera (transform monde→écran getScreenX/Y const), SpriteBatch (DrawAll avec
             caméra + std::optional srcRect, deux overloads Draw).
             Concepts vus : forward declaration vs include, const T& vs T par valeur,
             std::optional has_value() + opérateur bool, SDL3_image IMG_LoadTexture,
             SDL_GetTextureSize (struct opaque), CMAKE_CXX_USE_RESPONSE_FILE_FOR_INCLUDES,
             option() CMake pour désactiver tests submodule, git submodule add.
             Prochaine session : eliott-physics ou eliott-input.

[2026-02-23] Session 6 — eliott-input démarré.
             Décisions d'architecture : ActionMap<T> générique (mécanisme moteur, contenu jeu),
             KeyBinding = struct wrappant vector<Input>, Input = std::variant<SDL_Scancode, int, SDL_GamepadButton>.
             ActionMap<T> stocke unordered_map<T, vector<KeyBinding>> pour gérer combos (ET) et alternatives (OU).
             InputManager implémenté : SDL3, namespace ee::input, KeyState struct,
             singleton (Meyer's), SDL_Scancode comme clé (layout-indépendant), const sur les queries.
             Concepts vus : SDL2→SDL3 migration (events, keysym supprimé), scancode vs keycode,
             std::variant, if(auto it = map.find(k); it != map.end()), précédence des opérateurs,
             operator[] non-const sur unordered_map.

[2026-02-24] Session 7 — eliott-input complété.
             ActionMap<T> finalisée : bind(), isActive() avec std::visit + if constexpr.
             InputManager : constructeur ajouté (m_mouseX/Y init, SDL_Init(GAMEPAD)),
             gamepad implémenté (IsKeyDown/Held/Released SDL_GamepadButton, getAxisForce),
             callback onGamepadConnected, détection manettes déjà connectées via SDL_EVENT_GAMEPAD_ADDED.
             CMakeLists.txt écrit (SDL3, doctest, add_library STATIC, add_executable sans PRIVATE).
             Tests doctest écrits et passent : singleton getInstance, état initial clavier/souris,
             ActionMap bind (getSize), ActionMap isActive sans event SDL.
             Concepts vus : enum class vs enum, std::visit, if constexpr, std::is_same_v<decltype(val), T>,
             polling pattern, référence membre = singleton, default: return false dans switch de lambda,
             target_link_libraries PUBLIC vs PRIVATE, add_executable sans PRIVATE.

[2026-02-24] Session 8 — Décision audio + préparation eliott-audio.
             Backend audio décidé : SDL3_mixer (SDL2_mixer non compatible avec SDL3, ABI conflict).
             Eliott a une base AudioManager SDL2_mixer fonctionnelle à migrer.
             Concepts identifiés pour la migration :
             - include: SDL2/SDL_mixer.h → SDL3_mixer/SDL_mixer.h
             - Mix_OpenAudio signature changée : prend deviceID + SDL_AudioSpec* en SDL3
             - MIX_DEFAULT_FORMAT remplacé par types SDL3 audio (SDL_AUDIO_S16, etc.)
             - Reste de l'API (Mix_LoadMUS, Mix_PlayMusic, Mix_FreeMusic, Mix_Chunk,
               Mix_PlayChannel, Mix_VolumeChunk, Mix_FadeInMusic...) quasi-identique.
             Prochaine session : eliott-audio (nouveau repo).

[2026-02-25] Session 10 — Archi eliott-physics + eliott-math définie.
             Décisions actées : Transform dans eliott-math (partagé renderer+physics),
             Sprite dans eliott-renderer (dépend SDL3), RigidBody + Collider dans eliott-physics.
             Collider = std::variant<AABB, Circle> (formes supportées : rectangle + cercle).
             PhysicsWorld interroge ee::ecs::World, pattern ECS pur (séparation Transform/RigidBody).
             Principe validé : "you don't pay for what you don't use" — séparation des responsabilités.
             Prochaine étape : créer eliott-math (Vector2, Rect, Transform).

[2026-02-25] Session 11 — eliott-math complété.
             Créé : Vector2<T> (template, opérateurs arithmétiques, Magnitude, Normalize, Distance,
             explicit operator B()), Rect<T> (template, isInside, Intersects via anchor getPosition,
             explicit operator Rect<U>()), Transform (struct, position/scale/rotation, valeurs par défaut).
             CMakeLists.txt INTERFACE (lib header-only), doctest, triplet x64-mingw-static.
             Tests doctest écrits et passent tous (isInside, Intersects, Magnitude).
             Concepts vus : INTERFACE library CMake, = default, member initializers in struct,
             explicit operator Rect<U>() vs operator B(), static_cast<Vector2<U>> chaîné,
             triple pythagore pour tester Magnitude sur entiers, casse des includes sur Linux.
             Note : Magnitude() nommée "Magnetude" (faute conservée pour cohérence interne).
             Prochaine session : eliott-physics (RigidBody, Collider, Quadtree).

[2026-02-25] Session 9 — eliott-audio complété (sans tests).
             SDL3_mixer abandonné (non dispo vcpkg) → miniaudio single-header choisi.
             Architecture : AudioManager (singleton, wraps ma_engine, vector<ma_sound> slot-based),
             AudioMap<Event, Group> (template haut niveau, pattern ActionMap, random anti-répète).
             Concepts vus : ma_engine/ma_sound/ma_sound_group init/uninit, SoundHandle = size_t index,
             PCM frames (44100/s), ma_sound_set_fade_in_pcm_frames, ma_sound_set_stop_time_in_pcm_frames,
             operator[] unordered_map insère avant lecture size() (piège off-by-one),
             single-header = target_include_directories uniquement (pas find_package/link),
             IntelliSense header standalone → inclure depuis un .cpp compilé.
             Prochaine session : eliott-physics (Quadtree + RigidBody).
```
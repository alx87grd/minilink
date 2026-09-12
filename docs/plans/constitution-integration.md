# Plan: Consolidation de la Documentation (Constitution, Règles et Architecture)

**Statut :** Brouillon révisé pour revue par le mainteneur (2026-09-12).  
**Objectif :** Éliminer la redondance et le bruit normatif entre les documents racine de Minilink en établissant une séparation claire et sans chevauchement entre la philosophie, les règles de code, le comportement des agents et les contrats techniques.

---

## 1. Nouvelle architecture documentaire cible

Chaque document répond à **une seule question précise** pour un public clairement identifié :

| Document | Public | Rôle & Contenu unique | Ce qu'il ne contient plus |
| :--- | :--- | :--- | :--- |
| **`CONSTITUTION.md`** | Humains & Agents | **La boussole suprême :** Vision du *Unified Systems Lab*, contrat mathématique continu $\dot{x}=f(x,u,t;p)$, pureté fonctionnelle (#1), fermeture de composition (#2), promesse des deux voies (enseignement/recherche), simplicité et beauté du code. | Pas de règles de syntaxe Python, pas de commandes de linter ou de CI. |
| **`RULES.md`** | Humains & Agents | **L'échelle de revue en 7 questions :** Toutes les règles concrètes, testables et vérifiables (placement, API, textbook style, bare signatures, idiome `xp`, no `self.`, no `_` pseudo-privé, démos open-and-run). | Pas de spéculation philosophique ; pas de consignes d'interaction agent. |
| **`AGENTS.md`** | Agents IA | **Guide comportemental et workflow de l'agent :** Règles d'interaction avec l'utilisateur, préservation stricte des modifications utilisateur, périmètre des initiatives autonomes vs "ask first", porte CI locale (`ruff`, `pytest`), pas de polling CI. | Ne répète plus les règles de code (renvoie à `RULES.md` et `CONSTITUTION.md`). |
| **`DESIGN.md`** | Développeurs & Agents | **Contrats techniques d'implémentation :** Spécifications d'ingénierie logicielle (structure de `System` et `DiagramSystem`, ports, compilation JAX, §8 call chains). | Ne répète plus la philosophie générale ni les règles de style. |
| **`ROADMAP.md`** | Maintainer & Utilisateurs | **Plan de marche et maturité :** Jalons, versions, registre de maturité TRL, phases, queue de revue. | Ne redéfinit plus les contrats mathématiques. |

---

## 2. État d'avancement

- [x] **`CONSTITUTION.md` révisé et validé (101 lignes) :**
  - Retrait des noms de cours et des logiciels concurrents.
  - Priorité absolue accordée à la pureté de l'équation $f$ (#1) et fermeture algébrique (#2).
  - Suppression de la règle 4.6 (consolidation déplacée dans RULES).
  - Intégration du principe de beauté et simplicité (« *Simplicity is the ultimate sophistication* »).
  - Clarification de $T = tf(x, u, t; p)$ : poses et géométries strictement déclaratives, sans moteur de rendu ni boucle d'affichage.
- [x] **`RULES.md` créé et structuré (244 lignes) :**
  - Organisé selon l'échelle de revue à 7 questions ordonnées par coût d'erreur décroissant.
  - Règles universelles pour humains et agents, testables et concrètes (bare signatures, idiome `xp`, dual params, convention SI/radians, topologie de rétroaction, validation anticipée des ports).

---

## 3. Étapes d'exécution proposées

### Étape 1 : Élaguer et recentrer `AGENTS.md`
- **Ce qui est retiré :**
  - Section « Core directives » (maintenant dans `RULES.md`).
  - Section « Textbook style » (les 13 règles, style général, nommage mathématique $\to$ maintenant dans `RULES.md` §5).
  - Règle des deux voies (déjà dans `CONSTITUTION.md` §3 et `RULES.md` §3.3).
- **Ce qui reste (le rôle propre d'AGENTS.md) :**
  - Phrase d'ouverture pointant vers `CONSTITUTION.md` (vision) et `RULES.md` (règles de code).
  - **Comportement avec l'utilisateur :** Préservation sacrée des modifications utilisateur (scripts, tuning, scratch), coaching d'architecture.
  - **Périmètres d'autonomie :** Ce que l'agent fait directement vs ce qui nécessite l'accord du mainteneur (*maintainer-owned* vs *agent-managed*).
  - **Workflow & porte CI locale :** La séquence obligatoire avant push (`ruff check .`, `ruff format --check .`, `pytest` proportionné), sans polling CI inutile.
  - Procédure de passe de révision (*revision pass*).

### Étape 2 : Élaguer `DESIGN.md` (de ~1150 à ~400 lignes)
- **Ce qui est retiré :**
  - §1 « Design Principles » et « Product identity & scope » (intégralement couverts par `CONSTITUTION.md` et `RULES.md` §1).
  - Les descriptions redondantes qui répètent le code sans apporter de contrat.
- **Ce qui reste :**
  - §2 « Interface Layers » (politique des imports publics).
  - §3 « Package Map » (loi de dépendance et algorithme de placement).
  - §4 « Core Object Contracts » (promesses techniques de `System`, ports, paramètres, `DiagramSystem`).
  - §5 « Compilation and simulation » (paliers de compilation JAX et spécifications de `Simulator`).
  - §6 « Optimization and planning » (contrat problème / solveur / résultat).
  - §7 « Graphics and benchmarks ».
  - §8 « Call chains » (conservé intégralement car précieux et non trivial à déduire du code).

### Étape 3 : Nettoyer `ROADMAP.md`
- Remplacer le texte redondant du §2 (contrat des deux voies) par un renvoi direct vers `CONSTITUTION.md` §3 et `RULES.md` §3.3.
- Conserver intacts le registre TRL, les phases et les jalons de versions.

### Étape 4 : Validation globale
- Vérifier qu'aucun lien interne n'est brisé.
- Valider la suite de tests et la porte de qualité (`ruff check .`, `ruff format --check .`, `pytest`).

---

## 4. Points soumis à l'arbitrage du mainteneur

1. **Format d'`AGENTS.md` :** Es-tu d'accord pour qu'il soit concentré à 100 % sur les consignes comportementales de l'agent en renvoyant à `RULES.md` pour le style de code ?
2. **Sortie de `DESIGN.md` :** Valides-tu le maintien du fichier `DESIGN.md` ramené à ~400 lignes de purs contrats techniques (au lieu de le renommer ou de le fusionner) ?
